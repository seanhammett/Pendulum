'use strict';

const DT_MAX = 0.001;    // coarsest integration step (s)
const MAX_FRAME = 0.05;  // clamp on real time consumed per frame (s)
// Integration steps per frame, shared by the whole world rather than allowed per
// chain. Covers the worst corner of the input range (~17k for one chain of tiny
// light bobs at maximum friction) so the flight clock cannot fall behind real
// time. Three chains there cost ~7.2 ms; at the defaults, ~0.17 ms.
const MAX_STEPS = 20000;

// Longest chain offered. Parameter arrays are always this long and n says how
// much is in use, so changing link count never resizes them.
const MAX_LINKS = 3;

// Pendulums the page has: three tabs, three colours, three trail rows, all
// present at all times. How many hang from the pivot is `world`, below.
const MAX_PENDULUMS = 3;

// Each pendulum at load. All three divide the same 0.994 m of rod (the seconds
// pendulum; see index.html beside L1) into one, two and three links, so they
// differ in architecture alone and reach the same circle — the stage does not
// rescale as they are switched on and off. Unused links hold C's lengths, so
// extending A or B finds a link of the same family. A's numbers are in the
// markup too, and are read back from it at the foot of this file.
const DEFAULTS = [
  { n: 1, L: [0.994, 0.33133, 0.21767], m: [2, 1.8, 1] }, // A — simple
  { n: 2, L: [0.5, 0.494, 0.21767], m: [4, 2, 1] }, // B — double
  { n: 3, L: [0.445, 0.33133, 0.21767], m: [4, 1.8, 1] } // C — triple
];

// The aircraft, not the sculpture: one gravity, written by the flight profile
// and read by every chain.
const env = { g: 9.81 };

// --- The world -------------------------------------------------------------

// The chains do not interact: the shared pivot is a fixed point and not a body,
// so three chains are three independent Lagrangian systems drawn on one canvas.
// Everything here is per chain except env.g, the release time and the clock.
//
// slot is the chain's identity — colour, tab, trail row and readout all key off
// it, and it never changes.
function makeChain(slot) {
  const d = DEFAULTS[slot];
  return {
    slot,
    n: d.n, // links in the chain: 1, 2 or 3 — a simple, double or triple pendulum
    // Point masses m_i (kg) on massless rods L_i (m), copied out of DEFAULTS so
    // editing a pendulum cannot rewrite the table. b is this chain's viscous
    // friction at every hinge (N·m·s/rad); 0 is the ideal pendulum.
    L: d.L.slice(),
    m: d.m.slice(),
    b: 0.001,
    // Initial conditions in degrees, the units the boxes use; converted on reset.
    th0: [90, 0, 0],
    om0: [0, 0, 0],
    // [θ_0 … θ_{n-1}, ω_0 … ω_{n-1}], angles from straight down, CCW positive.
    // Sized to this chain's n, since the ω half starts at index n.
    s: new Float64Array(2 * d.n),
    // Real time banked but not yet integrated (s). Per chain, since chains run
    // at their own step sizes.
    pending: 0,
    // Simulated time this chain has reached (s). Chains sit up to one of their
    // own steps apart, worst measured 232 µs.
    t: 0,
    // Each traced bob's path as world-space [x, y, t], indexed by bob, so a
    // trail survives a change of view scale. Entry 0 is filled only on a single
    // pendulum; on a longer chain that bob's circle is its own rod's sweep.
    trails: [[], [], []],
    // The same points kept longer for the exporter; see the tape section.
    // Allocated on first write.
    tape: [null, null, null]
  };
}

// Every pendulum, one per slot, for the life of the page. Never added to or
// removed from: switching one off takes it out of `world` and leaves everything
// else about it intact, so switching it back on returns the same pendulum.
const chains = Array.from({ length: MAX_PENDULUMS }, (_, slot) => makeChain(slot));

// The chains hanging from the pivot: a dense subset of `chains`, ordered by
// slot, walked by the frame loop, the drawing and the exporter. Dense because
// the step budget indexes its scratch by position in here.
//
// Never empty — the last chain hanging cannot be switched off; see
// paintSelection.
const world = [chains[0]];
let sel = 0; // slot of the pendulum the panel is editing, hanging or not

// Whether this chain is on the pivot. Membership of world is the whole answer.
const hanging = (c) => world.includes(c);

// Chains are lettered where bobs are numbered, so B2 names a trace without
// having to say which kind of number it holds. Slot 0 is A everywhere,
// including the SVG header.
const CHAIN_NAME = ['A', 'B', 'C'];
const chainName = (c) => CHAIN_NAME[c.slot];

let running = false;
// The world clock, taken from the chain furthest along. One clock, one trail
// window, one flight.
let simTime = 0;
let lastFrame = 0;

let trailSeconds = 25;

// --- Physics ---------------------------------------------------------------

// Equations of motion for an n-link chain, as the linear system
// A(θ)·θ'' = r(θ, θ') solved directly, so friction enters as a generalised
// force Q_i. Writing M_i for the mass at or below joint i and c_ij for
// M_max(i,j)·L_i·L_j,
//
//   A_ij = c_ij·cos(θ_i − θ_j)
//   r_i  = −Σ_j c_ij·sin(θ_i − θ_j)·ω_j²  −  M_i·g·L_i·sin(θ_i)  +  Q_i
//
// n = 2 is the familiar double pendulum term for term; n = 1 collapses to
// θ'' = −(g/L)·sin θ with friction, as the general case with nothing to sum
// over. A is JᵀMJ for the Jacobian of the bob positions, so symmetric positive
// definite for any positive masses and lengths, which the control minimums
// guarantee.
//
// Every function here takes its chain, and is pure but for the scratch.

// Scratch, one set per link count, shared by every chain of that size. Chains
// step one at a time and nothing here survives a call.
function makeKernel(n) {
  const N = 2 * n;
  return {
    A: Array.from({ length: n }, () => new Float64Array(n)),
    rhs: new Float64Array(n),
    acc: new Float64Array(n),
    tail: new Float64Array(n),
    k1: new Float64Array(N),
    k2: new Float64Array(N),
    k3: new Float64Array(N),
    k4: new Float64Array(N),
    mid: new Float64Array(N)
  };
}

const KERNEL = { 1: makeKernel(1), 2: makeKernel(2), 3: makeKernel(3) };

// tail[i] = Σ_{k ≥ i} m_k, the mass hanging at or below joint i. Accumulated
// from the tip so the whole chain costs one pass.
function masses(c) {
  const tail = KERNEL[c.n].tail;
  let sum = 0;
  for (let i = c.n - 1; i >= 0; i--) {
    sum += c.m[i];
    tail[i] = sum;
  }
}

// Writes dθ/dt and dω/dt for the state s into out, packed the same way.
function derivatives(c, s, out) {
  const n = c.n;
  const L = c.L;
  const b = c.b;
  const g = env.g;
  const k = KERNEL[n];
  const A = k.A;
  const rhs = k.rhs;
  const tail = k.tail;
  const acc = k.acc;
  masses(c);

  for (let i = 0; i < n; i++) {
    rhs[i] = -tail[i] * g * L[i] * Math.sin(s[i]);
    A[i][i] = tail[i] * L[i] * L[i];
  }

  // Only the i < j half is computed: cos(θ_i − θ_j) is symmetric and
  // sin(θ_i − θ_j) antisymmetric, so each pair of links costs one sin and one
  // cos rather than four.
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      const cij = tail[j] * L[i] * L[j]; // tail[max(i, j)], and j > i here
      const d = s[i] - s[j];
      const sinD = Math.sin(d);
      A[i][j] = A[j][i] = cij * Math.cos(d);
      rhs[i] -= cij * sinD * s[n + j] * s[n + j];
      rhs[j] += cij * sinD * s[n + i] * s[n + i];
    }
  }

  // Viscous friction at every hinge, from the Rayleigh dissipation function
  // F = ½b[ω_0² + Σ(ω_i − ω_{i-1})²] with Q_i = −∂F/∂ω_i: the pivot, then each
  // elbow. F ≥ 0, so this can only remove energy, and a third link adds a third
  // term, so the same b drains a triple faster than a double.
  for (let i = 0; i < n; i++) {
    const prev = i === 0 ? s[n] : s[n + i] - s[n + i - 1];
    const next = i === n - 1 ? 0 : s[n + i + 1] - s[n + i];
    rhs[i] += b * (next - prev);
  }

  solve(c);

  for (let i = 0; i < n; i++) {
    out[i] = s[n + i];
    out[n + i] = acc[i];
  }
}

// Solves A·acc = rhs. The 1x1 and 2x2 have closed forms; anything larger goes
// to Cholesky below.
function solve(c) {
  const n = c.n;
  const k = KERNEL[n];
  const A = k.A;
  const rhs = k.rhs;
  const acc = k.acc;

  if (n === 1) {
    // A is the moment of inertia about the pivot, m·L², positive by the
    // controls' own minimums.
    acc[0] = rhs[0] / A[0][0];
    return;
  }

  if (n === 2) {
    // det = m2·L1²·L2²·(m1 + m2·sin²Δ), non-zero for any m1 > 0.
    const det = A[0][0] * A[1][1] - A[0][1] * A[0][1];
    acc[0] = (rhs[0] * A[1][1] - A[0][1] * rhs[1]) / det;
    acc[1] = (A[0][0] * rhs[1] - A[0][1] * rhs[0]) / det;
    return;
  }

  // Cholesky in place on A's lower triangle, then forward and back
  // substitution. No pivoting, and the square root can never see a negative,
  // because A is positive definite.
  for (let i = 0; i < n; i++) {
    for (let j = 0; j <= i; j++) {
      let sum = A[i][j];
      for (let q = 0; q < j; q++) sum -= A[i][q] * A[j][q];
      if (i === j) A[i][i] = Math.sqrt(sum);
      else A[i][j] = sum / A[j][j];
    }
  }
  for (let i = 0; i < n; i++) {
    let sum = rhs[i];
    for (let q = 0; q < i; q++) sum -= A[i][q] * acc[q];
    acc[i] = sum / A[i][i];
  }
  for (let i = n - 1; i >= 0; i--) {
    let sum = acc[i];
    for (let q = i + 1; q < n; q++) sum -= A[q][i] * acc[q];
    acc[i] = sum / A[i][i];
  }
}

// Rate at which friction drains this chain's mechanical energy: dE/dt = -2F ≤ 0.
function dissipation(c) {
  const n = c.n;
  const s = c.s;
  let sum = s[n] * s[n];
  for (let i = 1; i < n; i++) {
    const rel = s[n + i] - s[n + i - 1];
    sum += rel * rel;
  }
  return -c.b * sum;
}

// Classic 4th-order Runge-Kutta, over the preallocated stage buffers.
function step(c, h) {
  const N = 2 * c.n;
  const s = c.s;
  const k = KERNEL[c.n];
  const { k1, k2, k3, k4, mid } = k;
  derivatives(c, s, k1);
  for (let i = 0; i < N; i++) mid[i] = s[i] + k1[i] * (h / 2);
  derivatives(c, mid, k2);
  for (let i = 0; i < N; i++) mid[i] = s[i] + k2[i] * (h / 2);
  derivatives(c, mid, k3);
  for (let i = 0; i < N; i++) mid[i] = s[i] + k3[i] * h;
  derivatives(c, mid, k4);
  for (let i = 0; i < N; i++) {
    s[i] += (h / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
  }
}

// Fills A with the mass matrix at the current angles, for callers that want it
// on its own. derivatives() rebuilds A from scratch every time, so overwriting
// what it left behind is safe.
function buildA(c) {
  const n = c.n;
  const L = c.L;
  const s = c.s;
  const A = KERNEL[n].A;
  const tail = KERNEL[n].tail;
  masses(c);
  for (let i = 0; i < n; i++) {
    A[i][i] = tail[i] * L[i] * L[i];
    for (let j = i + 1; j < n; j++) {
      A[i][j] = A[j][i] = tail[j] * L[i] * L[j] * Math.cos(s[i] - s[j]);
    }
  }
}

// Upper bound on the square of the fastest small-oscillation frequency:
// ω_max² ≤ tr(A⁻¹K) with K = diag(M_i·|g|·L_i). The trace overshoots the true
// largest eigenvalue by at most a factor n, and in practice by 1.0–1.3x.
// Assumes buildA() and masses() have just run for this chain.
function stiffness(c) {
  const n = c.n;
  const L = c.L;
  const A = KERNEL[n].A;
  const tail = KERNEL[n].tail;
  const gk = Math.abs(env.g);
  const K0 = tail[0] * gk * L[0];

  // One link has one mode, so the trace is not an upper bound but the answer:
  // m·g·L over m·L² is g/L, the small-oscillation frequency itself. Computed
  // before K1, which would read past the end of a one-element tail.
  if (n === 1) return K0 / A[0][0];

  const K1 = tail[1] * gk * L[1];

  if (n === 2) {
    const det = A[0][0] * A[1][1] - A[0][1] * A[0][1];
    return (K0 * A[1][1] + K1 * A[0][0]) / det;
  }

  // Diagonal of A⁻¹ as cofactors over the determinant, for a symmetric 3x3.
  const K2 = tail[2] * gk * L[2];
  const c00 = A[1][1] * A[2][2] - A[1][2] * A[1][2];
  const c11 = A[0][0] * A[2][2] - A[0][2] * A[0][2];
  const c22 = A[0][0] * A[1][1] - A[0][1] * A[0][1];
  const det = A[0][0] * c00
    - A[0][1] * (A[0][1] * A[2][2] - A[1][2] * A[0][2])
    + A[0][2] * (A[0][1] * A[1][2] - A[1][1] * A[0][2]);
  return (K0 * c00 + K1 * c11 + K2 * c22) / det;
}

// How fine this chain's integration step has to be, from its own state: the
// fast normal mode runs away when an upper bob is light relative to what hangs
// below it. Current rotation rates are added, so a fast spin tightens the step
// too. Per chain, since the chains do not interact.
function stepSize(c) {
  const n = c.n;
  const L = c.L;
  const m = c.m;
  const b = c.b;
  const s = c.s;
  buildA(c);
  const wSq = Math.max(0, stiffness(c));

  // Friction adds its own rate, b/I, which gets stiff for a strong damper on a
  // light bob. Zero when b is zero, so the ideal pendulum is unaffected.
  let inertia = Infinity;
  let speeds = 0;
  for (let i = 0; i < n; i++) {
    inertia = Math.min(inertia, m[i] * L[i] * L[i]);
    speeds += Math.abs(s[n + i]);
  }

  const w = Math.sqrt(wSq) + speeds + (2 * b) / inertia;
  return Math.min(DT_MAX, 0.004 / w);
}

// Energy of the motion alone, ½ωᵀAω — each bob rides on the ones above it, so
// its speed is the vector sum of every rod's contribution, not just L_i·ω_i.
function kinetic(c) {
  const n = c.n;
  const L = c.L;
  const s = c.s;
  const tail = KERNEL[n].tail;
  masses(c);
  let ke = 0;
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      ke += tail[i > j ? i : j] * L[i] * L[j]
        * Math.cos(s[i] - s[j]) * s[n + i] * s[n + j];
    }
  }
  return 0.5 * ke;
}

// Gravitational energy at the current g, measured from the rest configuration —
// every rod hanging straight down — rather than from the pivot: Σm·g·y shifted
// by the constant −Σ M_i·g·L_i. So it reads zero at rest, is never negative
// under positive g, and is exactly zero at 0g whatever the attitude. The datum
// is set by g, L and m, so changing any of those moves the zero point.
function potential(c) {
  const n = c.n;
  const L = c.L;
  const s = c.s;
  const g = env.g;
  const tail = KERNEL[n].tail;
  masses(c);
  let pe = 0;
  for (let i = 0; i < n; i++) pe += tail[i] * g * L[i] * (1 - Math.cos(s[i]));
  return pe;
}

// Total mechanical energy of one chain. Constant at b = 0 while its parameters
// are held still, which is the check on the integration.
function energy(c) {
  return kinetic(c) + potential(c);
}

// Bob positions, cumulative down the chain, into a reused array of pairs. The
// caller has to be done with them before the next chain asks.
const POS = Array.from({ length: MAX_LINKS }, () => [0, 0]);

function positions(c) {
  const L = c.L;
  const s = c.s;
  let x = 0;
  let y = 0;
  for (let i = 0; i < c.n; i++) {
    x += L[i] * Math.sin(s[i]);
    y -= L[i] * Math.cos(s[i]);
    POS[i][0] = x;
    POS[i][1] = y;
  }
  return POS;
}

// --- Step budget -----------------------------------------------------------

// MAX_STEPS is shared by the whole world, water-filled rather than split
// equally: chains wanting less than an equal share hand the surplus back, and it
// goes to those that want more. So the frame stays bounded whatever the chain
// count, a chain that cannot keep up drops its own backlog and not the world's,
// and one stiff chain does not slow the others. At one pendulum it reduces to
// min(want, MAX_STEPS).
//
// Sized once; at three chains the sort below is an insertion sort over three.
const stepH = new Float64Array(MAX_PENDULUMS);
const stepWant = new Float64Array(MAX_PENDULUMS);
const stepCap = new Float64Array(MAX_PENDULUMS);
const stepOrder = new Int32Array(MAX_PENDULUMS);

function allocate(k) {
  for (let i = 0; i < k; i++) stepOrder[i] = i;
  for (let i = 1; i < k; i++) {
    const v = stepOrder[i];
    let j = i - 1;
    while (j >= 0 && stepWant[stepOrder[j]] > stepWant[v]) {
      stepOrder[j + 1] = stepOrder[j];
      j--;
    }
    stepOrder[j + 1] = v;
  }

  // Cheapest first, so each chain in turn is offered an equal share of what is
  // left and the ones that decline it fund the ones that do not.
  let left = MAX_STEPS;
  for (let r = 0; r < k; r++) {
    const i = stepOrder[r];
    const share = Math.floor(left / (k - r));
    stepCap[i] = Math.min(stepWant[i], share);
    left -= stepCap[i];
  }
}

// Steps every chain by the real time it has banked, inside that one budget.
// Per-chain step sizes leave the chains at slightly different simulated times —
// worst measured 232 µs, 0.7 mm of tip travel on a 1.2 m reach, and exactly
// zero between chains with identical parameters.
function advance() {
  const k = world.length;

  for (let i = 0; i < k; i++) {
    const c = world[i];
    const h = stepSize(c);
    stepH[i] = h;
    // Rounded up, so a chain is never told it wanted fewer steps than the loop
    // below takes, and dropped into slow motion over a rounding error.
    stepWant[i] = Math.ceil(c.pending / h);
  }
  allocate(k);

  for (let i = 0; i < k; i++) {
    const c = world[i];
    const h = stepH[i];
    const cap = stepCap[i];
    let steps = 0;
    while (c.pending >= h && steps < cap) {
      step(c, h);
      c.pending -= h;
      c.t += h;
      steps++;
    }
    // Too fast to integrate in real time: this chain drops its backlog rather
    // than let it snowball. It runs in slow motion but stays correct, and the
    // chains that kept up are left alone.
    if (steps === cap && cap < stepWant[i]) c.pending = 0;
  }
}

// --- Flight profile --------------------------------------------------------

// One parabolic-flight cycle, in multiples of Earth gravity. Contiguous, sums
// to exactly CYCLE seconds, and ends where it starts so the loop is seamless.
const G_EARTH = 9.81;
const CYCLE = 180;

// Trims the 93 s of level flight after the second hypergravity phase to 10 s.
// Nothing about the integration changes; there are simply fewer seconds of it
// before the next pull-up.
const BASELINE_SHORT = 10;
const CYCLE_SHORT = 87 + BASELINE_SHORT;
let shortcut = false;
const cycle = () => (shortcut ? CYCLE_SHORT : CYCLE);

// The last phase runs to the full 180 s; the shortcut wraps early instead of
// rewriting it, which is exact because that phase is flat at 1g throughout.
// key looks the phase name up in the string table; the two hypergravity phases
// share one, as they are the same thing happening twice.
const PHASES = [
  { t0: 0,   t1: 5,   from: 1,   to: 1,   key: 'baseline' },
  { t0: 5,   t1: 10,  from: 1,   to: 1.8, key: 'pullup' },
  { t0: 10,  t1: 30,  from: 1.8, to: 1.8, key: 'hyper' },
  { t0: 30,  t1: 35,  from: 1.8, to: 0,   key: 'injection' },
  { t0: 35,  t1: 57,  from: 0,   to: 0,   key: 'apesanteur' },
  { t0: 57,  t1: 62,  from: 0,   to: 1.8, key: 'pullout' },
  { t0: 62,  t1: 82,  from: 1.8, to: 1.8, key: 'hyper' },
  { t0: 82,  t1: 87,  from: 1.8, to: 1,   key: 'recovery' },
  { t0: 87,  t1: 180, from: 1,   to: 1,   key: 'baseline' }
];

let flightOn = false;
let flightTime = 0;
// Point in the cycle at which the sculpture is let go, shared by every chain, as
// gravity is: one aircraft, one moment of release. Before it the piece is
// clamped at its initial condition while the aircraft flies the profile around
// it; after it, free for the rest of the flight. Confined to one cycle, since
// releasing at t in a later parabola is identical to releasing at t in the first.
let release = 0;
// The run starts this far ahead of the release, so a late release costs a second
// of clamped lead-in rather than minutes.
const LEAD_IN = 1;
const startTime = () => Math.max(0, release - LEAD_IN);

// Gravity level at time t in the cycle, eased through the 5 s transitions with
// a smoothstep so the rate of change starts and ends at zero.
function flightAt(t) {
  const C = cycle();
  // Folded with a conditional rather than ((t % C) + C) % C, which rounds
  // differently for different C and would make the shortened cycle disagree
  // with the full one by an ULP over the span they share.
  let u = t % C;
  if (u < 0) u += C;
  const ph = PHASES.find((p) => u < p.t1) || PHASES[PHASES.length - 1];
  const x = (u - ph.t0) / (ph.t1 - ph.t0);
  const s = x * x * (3 - 2 * x);
  return { u, ph, level: ph.from + (ph.to - ph.from) * s };
}

// --- Language --------------------------------------------------------------

// French by default, and not persisted, so every load opens in French.
let lang = 'fr';

// Falls back to English rather than showing a raw key.
const t = (key) => STRINGS[lang][key] ?? STRINGS.en[key] ?? key;

// Fills {name} placeholders from an object of values.
const fmt = (key, vals) => t(key).replace(/\{(\w+)\}/g, (_, k) => vals[k]);

const phaseName = (ph) => t('phase.' + ph.key);

// The same name inside a sentence: no capital, no direction arrow.
const phaseInline = (ph) => phaseName(ph).replace(/ [↗↘]$/, '').toLowerCase();

// --- Rendering -------------------------------------------------------------

const canvas = document.getElementById('stage');
const wrap = document.getElementById('stage-wrap');
const ctx = canvas.getContext('2d');
let size = 400;

function sizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  size = Math.max(240, Math.floor(Math.min(wrap.clientWidth, wrap.clientHeight)) - 2);
  canvas.style.width = size + 'px';
  canvas.style.height = size + 'px';
  canvas.width = Math.round(size * dpr);
  canvas.height = Math.round(size * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

new ResizeObserver(sizeCanvas).observe(wrap);

// The stage's palette, read from the same custom properties as the panel's, so
// the colours are defined once — in the stylesheet.
const css = getComputedStyle(document.documentElement);
const prop = (name) => css.getPropertyValue(name).trim();
const COLOUR = {
  // The pivot belongs to the world rather than any one chain, so it is neutral.
  pivot: prop('--text'),
  // Rods are white for every chain; the chain's colour runs down their core.
  rod: prop('--rod'),
  // Panel chrome, used by the flight profile.
  accent: prop('--accent'),
  // Indexed by slot, not by position in the world.
  chain: [0, 1, 2].map((p) => prop('--p' + (p + 1)))
};

// The coloured core stroked down the centre of each white rod, as a fraction of
// the rod's width.
const CORE_RATIO = 0.34;

// The white ring round each bob, in CSS pixels, so it is the same apparent
// weight on any display. Flat rather than a fraction of the radius.
const BOB_RING = 2;

// Two ways of drawing the same recorded points:
//
//   'line' — the points joined, for the shape of the path. The default.
//   'dots' — one dot per rendered frame, not joined. Frames are evenly spaced
//            in time, so the gap between dots is the bob's speed.
//
// Either way the trail fades from transparent (oldest) to solid (newest), in a
// couple of dozen batched draws rather than one per point.
const TRAIL_CHUNKS = 24;
const TRAIL_DOT = 1.25;
let trailStyle = 'line';

function drawTrail(pts, colour, toX, toY) {
  if (!pts.length) return;
  const dots = trailStyle === 'dots';
  ctx.fillStyle = colour;
  ctx.strokeStyle = colour;
  ctx.lineWidth = 1.5;
  ctx.lineJoin = 'round';

  for (let c = 0; c < TRAIL_CHUNKS; c++) {
    const from = Math.floor((c * pts.length) / TRAIL_CHUNKS);
    const to = Math.floor(((c + 1) * pts.length) / TRAIL_CHUNKS);
    if (to <= from) continue;
    // The line starts each chunk one point early, so the segment spanning a
    // chunk boundary is drawn and the fade has no gaps. Dots need no overlap,
    // and repeating one would only darken it.
    const start = dots ? from : Math.max(0, from - 1);

    ctx.globalAlpha = ((c + 1) / TRAIL_CHUNKS) * 0.75;
    ctx.beginPath();
    for (let i = start; i < to; i++) {
      const x = toX(pts[i][0]);
      const y = toY(pts[i][1]);
      if (!dots) {
        if (i === start) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        continue;
      }
      // Start a fresh subpath at each dot, or the arcs are joined by the line
      // this style exists to do without.
      ctx.moveTo(x + TRAIL_DOT, y);
      ctx.arc(x, y, TRAIL_DOT, 0, Math.PI * 2);
    }
    if (dots) ctx.fill(); else ctx.stroke();
  }

  ctx.globalAlpha = 1;
}

function draw() {
  const bobR = Math.min(11, Math.max(5, size * 0.02));
  // Radius follows mass by volume, so an 8x heavier bob draws twice as wide.
  // Clamped, since the full 0.1-5 kg range would otherwise be extreme.
  const massR = (m) => bobR * Math.min(1.7, Math.max(0.6, Math.cbrt(m)));

  // Scaled to the largest reach in the world rather than to each chain's own,
  // so a 0.32 m pendulum draws smaller than a 1.20 m one.
  let reach = 0;
  let widest = 0;
  for (const c of world) {
    let r = 0;
    for (let i = 0; i < c.n; i++) {
      r += c.L[i];
      widest = Math.max(widest, massR(c.m[i]));
    }
    reach = Math.max(reach, r);
  }

  // Pivot centred, so the whole reachable circle always fits without clipping.
  const scale = (size / 2 - widest - 6) / reach;
  const cx = size / 2;
  const cy = size / 2;
  const toX = (x) => cx + x * scale;
  const toY = (y) => cy - y * scale;

  ctx.clearRect(0, 0, size, size);

  // Every trail before any rod, so no path buries a linkage; within a chain the
  // oldest bob first, so a slower bob's path does not sit on the tip's.
  for (const c of world) {
    for (let i = firstTraced(c); i < c.n; i++) {
      if (traced(c, i)) drawTrail(c.trails[i], COLOUR.chain[c.slot], toX, toY);
    }
  }

  // Rods are opaque white with a thin core in their chain's colour, which is
  // what tells two chains apart where they cross.
  const rodW = Math.max(2, size * 0.006);
  const coreW = Math.max(1, rodW * CORE_RATIO);
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';

  for (const c of world) {
    const p = positions(c);
    const colour = COLOUR.chain[c.slot];

    ctx.beginPath();
    ctx.moveTo(cx, cy);
    for (let i = 0; i < c.n; i++) ctx.lineTo(toX(p[i][0]), toY(p[i][1]));
    // Stroked twice over the one path, so the core costs a stroke and no more.
    ctx.lineWidth = rodW;
    ctx.strokeStyle = COLOUR.rod;
    ctx.stroke();
    ctx.lineWidth = coreW;
    ctx.strokeStyle = colour;
    ctx.stroke();

    // One colour for every bob in the chain, drawn after the rods so the rod
    // ends are capped, and ringed in the rod's white so the chain reads as one
    // object and two overlapping bobs stay separable.
    ctx.fillStyle = colour;
    ctx.strokeStyle = COLOUR.rod;
    ctx.lineWidth = BOB_RING;
    for (let i = 0; i < c.n; i++) {
      ctx.beginPath();
      ctx.arc(toX(p[i][0]), toY(p[i][1]), massR(c.m[i]), 0, Math.PI * 2);
      ctx.fill();
      // Stroked over the fill, so the ring straddles the radius and the mass
      // reads at its own size rather than its size plus a ring.
      ctx.stroke();
    }
  }

  // Last and once: the pivot belongs to the world, not to any one pendulum.
  ctx.fillStyle = COLOUR.pivot;
  ctx.beginPath();
  ctx.arc(cx, cy, bobR * 0.45, 0, Math.PI * 2);
  ctx.fill();
}

// The g(t) curve for one cycle, with a playhead at the current position.
const profile = document.getElementById('profile');
const pctx = profile.getContext('2d');

// A span of the cycle to mark on the profile, as [t0, t1] in cycle seconds, or
// null for nothing. export.js writes the seconds it is going to save here; a
// page without export.js leaves it null.
let markSpan = null;

// The curve is the same picture every frame — only the playhead moves — so it is
// painted once into an offscreen canvas and blitted, rather than re-sampled 241
// times and stroked twice per frame.
const layer = document.createElement('canvas');
const lctx = layer.getContext('2d');
let layerKey = '';

// Repaints the static half of the profile — the 1g reference, the fill and the
// curve — only when an input changes. The key names every one of them: canvas
// geometry, the cycle length the shortcut switches, and flightOn, which the
// curve is stroked a different colour for.
function buildLayer(w, h, dpr) {
  const key = w + '|' + h + '|' + dpr + '|' + cycle() + '|' + flightOn;
  if (key === layerKey) return;
  layerKey = key;

  layer.width = Math.round(w * dpr);
  layer.height = Math.round(h * dpr);
  lctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  lctx.clearRect(0, 0, w, h);

  const pad = 4;
  const top = pad;
  const bottom = h - pad;
  const C = cycle();
  const toY = (level) => bottom - (level / 1.8) * (bottom - top);

  // 1g reference line.
  lctx.strokeStyle = '#2c3242';
  lctx.setLineDash([3, 3]);
  lctx.lineWidth = 1;
  lctx.beginPath();
  lctx.moveTo(0, toY(1));
  lctx.lineTo(w, toY(1));
  lctx.stroke();
  lctx.setLineDash([]);

  // Sampled through flightAt, so the drawing and the physics cannot disagree
  // about the shape.
  const pts = [];
  for (let i = 0; i <= 240; i++) {
    const time = (i / 240) * C;
    pts.push([(time / C) * w, toY(flightAt(time).level)]);
  }

  lctx.beginPath();
  lctx.moveTo(pts[0][0], pts[0][1]);
  for (const [x, y] of pts) lctx.lineTo(x, y);
  lctx.lineTo(w, bottom);
  lctx.lineTo(0, bottom);
  lctx.closePath();
  lctx.fillStyle = COLOUR.accent;
  lctx.globalAlpha = 0.12;
  lctx.fill();
  lctx.globalAlpha = 1;

  lctx.beginPath();
  lctx.moveTo(pts[0][0], pts[0][1]);
  for (const [x, y] of pts) lctx.lineTo(x, y);
  lctx.strokeStyle = flightOn ? COLOUR.accent : '#565e73';
  lctx.lineWidth = 1.5;
  lctx.stroke();
}

function drawProfile() {
  const dpr = window.devicePixelRatio || 1;
  const w = profile.clientWidth;
  const h = profile.clientHeight;
  if (!w) return;
  if (profile.width !== Math.round(w * dpr)) {
    profile.width = Math.round(w * dpr);
    profile.height = Math.round(h * dpr);
  }
  pctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  pctx.clearRect(0, 0, w, h);

  buildLayer(w, h, dpr);
  // Drawn at CSS size under the same dpr transform the layer was painted with,
  // so it lands one device pixel to one device pixel rather than resampled.
  pctx.drawImage(layer, 0, 0, w, h);

  if (!flightOn) return;

  const pad = 4;
  const top = pad;
  const bottom = h - pad;
  const C = cycle();
  const toY = (level) => bottom - (level / 1.8) * (bottom - top);
  const toXt = (time) => (time / C) * w;

  // Shade the span during which the piece is clamped, and mark the release.
  if (release > 0) {
    const rx = toXt(release);
    pctx.fillStyle = 'rgba(18, 21, 29, 0.62)';
    pctx.fillRect(0, 0, rx, h);
    pctx.strokeStyle = '#8d94a8';
    pctx.setLineDash([2, 2]);
    pctx.lineWidth = 1;
    pctx.beginPath();
    pctx.moveTo(rx, 0);
    pctx.lineTo(rx, h);
    pctx.stroke();
    pctx.setLineDash([]);
  }

  // The span an export will cover, in the accent. Drawn over the clamped
  // shading, since a span chosen before the release is still the span saved.
  if (markSpan) {
    const a = toXt(clamp(markSpan[0], 0, C));
    const b = toXt(clamp(markSpan[1], 0, C));
    if (b > a) {
      pctx.fillStyle = COLOUR.accent;
      pctx.globalAlpha = 0.18;
      pctx.fillRect(a, top - 2, b - a, bottom - top + 4);
      pctx.globalAlpha = 1;
      pctx.strokeStyle = COLOUR.accent;
      pctx.lineWidth = 1;
      pctx.beginPath();
      // Half-pixel offsets, and inward, so both edges land on a pixel boundary
      // and a span at t = 0 keeps its left edge on the canvas.
      pctx.moveTo(a + 0.5, top - 2);
      pctx.lineTo(a + 0.5, bottom + 2);
      pctx.moveTo(b - 0.5, top - 2);
      pctx.lineTo(b - 0.5, bottom + 2);
      pctx.stroke();
    }
  }

  const { u, level } = flightAt(flightTime);
  const x = toXt(u);
  pctx.strokeStyle = '#ffb347';
  pctx.lineWidth = 1;
  pctx.beginPath();
  pctx.moveTo(x, top - 2);
  pctx.lineTo(x, bottom + 2);
  pctx.stroke();

  pctx.fillStyle = '#ffb347';
  pctx.beginPath();
  pctx.arc(x, toY(level), 3, 0, Math.PI * 2);
  pctx.fill();
}

const deg = (rad) => (rad * 180) / Math.PI;

function wrapDeg(rad) {
  let d = deg(rad) % 360;
  if (d > 180) d -= 360;
  if (d < -180) d += 360;
  return d;
}

const set = (id, text) => { document.getElementById(id).textContent = text; };

// State follows the selected tab; Energy shows all three chains at once, and
// names none of them in its legend. There is no all-chains energy total.
//
// A pendulum that is switched off is not being stepped, so what these show is it
// sitting at its initial conditions. The hollow dot in the State legend and on
// that pendulum's energy row is what says the numbers will not move.
function updateReadout() {
  const c = chains[sel];
  for (let i = 0; i < c.n; i++) {
    set('r-t' + (i + 1), wrapDeg(c.s[i]).toFixed(1) + '°');
    set('r-w' + (i + 1), deg(c.s[c.n + i]).toFixed(0) + '°/s');
  }
  set('r-time', simTime.toFixed(2) + ' s');

  // Split rather than summed: in flight the potential term drains to nothing at
  // 0g while the kinetic term carries on untouched.
  for (let p = 0; p < MAX_PENDULUMS; p++) {
    const chain = chains[p];
    const pe = potential(chain);
    const ke = kinetic(chain);
    const cell = E_CELL[p];
    setCell(cell[0], pe.toFixed(2));
    setCell(cell[1], ke.toFixed(2));
    setCell(cell[2], (pe + ke).toFixed(2));
  }
}

// Nine figures a frame, so a cell whose text has not changed is not written.
// The chains that are switched off are the case this is for: they are not being
// stepped, so their cells hold the same text for as long as they stay off.
function setCell(node, text) {
  if (node.textContent !== text) node.textContent = text;
}

// --- Frame cost ------------------------------------------------------------

// Two numbers for two questions. The gap between successive
// requestAnimationFrame timestamps is what the display delivered; the time spent
// inside frame() is what our own code accounts for. The difference is the
// browser's — style, layout, compositing and whatever else is on the machine.
const PERF_WINDOW = 60; // one second at full rate
const frameGap = new Float64Array(PERF_WINDOW);
const frameWork = new Float64Array(PERF_WINDOW);
let perfAt = 0;
let perfCount = 0;
let perfPrev = 0;
let perfShown = 0;

// Rewritten four times a second: at 60 Hz the digits change faster than they
// can be read.
const PERF_INTERVAL = 250;

function recordFrame(now, work) {
  // The first frame has no predecessor, and is skipped rather than logged as a
  // gap of the whole page load.
  if (perfPrev) {
    frameGap[perfAt] = now - perfPrev;
    frameWork[perfAt] = work;
    perfAt = (perfAt + 1) % PERF_WINDOW;
    if (perfCount < PERF_WINDOW) perfCount++;
  }
  perfPrev = now;

  if (now - perfShown < PERF_INTERVAL) return;
  perfShown = now;

  let gap = 0;
  let sum = 0;
  let worst = 0;
  for (let i = 0; i < perfCount; i++) {
    gap += frameGap[i];
    sum += frameWork[i];
    if (frameWork[i] > worst) worst = frameWork[i];
  }
  if (!perfCount) return;
  set('r-fps', (1000 / (gap / perfCount)).toFixed(0) + ' fps');
  // Mean and worst together: a steady 4 ms and a 4 ms mean with a 20 ms spike
  // read the same as a mean alone, and the spike is what drops a frame.
  set('r-cost', (sum / perfCount).toFixed(1) + ' / ' + worst.toFixed(1) + ' ms');
}

// --- The export tape -------------------------------------------------------
//
// A second record of the same points, sized for exporting rather than drawing.
// It cannot be one buffer with the display trail, which is redrawn in full every
// frame and so capped at about a minute, while the most recent completed
// instance of the 93 s baseline can begin 273 s back. Two cycles covers that, so
// a section stays exportable from the moment it has first been flown until the
// recording is cleared.
const TAPE_SECONDS = 2 * CYCLE;

// Sampled on the simulation clock rather than once per frame, so the capacity is
// a duration and not a frame count and a 120 Hz display records the same points
// as a 60 Hz one.
const TAPE_HZ = 60;
const TAPE_DT = 1 / TAPE_HZ;
const TAPE_CAP = Math.ceil(TAPE_SECONDS * TAPE_HZ);

// Flat [x, y, t] triples in a ring: 21 600 points, 518 KB, no allocation per
// point. Never pruned — the oldest falls off the far end as the newest is
// written.
const makeTape = () => ({ buf: new Float64Array(TAPE_CAP * 3), head: 0, len: 0, next: 0 });

function tapePush(tp, x, y, t) {
  const j = tp.head * 3;
  tp.buf[j] = x;
  tp.buf[j + 1] = y;
  tp.buf[j + 2] = t;
  tp.head = (tp.head + 1) % TAPE_CAP;
  if (tp.len < TAPE_CAP) tp.len++;
  // Advanced rather than set, so the sample rate does not drift with the frame
  // timing; resynced when the clock has already run past it, so a stall or a
  // rewound tape writes one point instead of a burst of catching up.
  tp.next += TAPE_DT;
  if (tp.next < t) tp.next = t;
}

// The ring read back oldest first, as [x, y, t] in the window [from, to] of this
// chain's own time. Boxed here because this runs once per export, not per frame.
function tapePoints(tp, from, to) {
  const out = [];
  if (!tp) return out;
  const start = (tp.head - tp.len + TAPE_CAP) % TAPE_CAP;
  for (let k = 0; k < tp.len; k++) {
    const j = ((start + k) % TAPE_CAP) * 3;
    const t = tp.buf[j + 2];
    if (t >= from && t <= to) out.push([tp.buf[j], tp.buf[j + 1], t]);
  }
  return out;
}

// How far back a tape actually reaches, or Infinity when it is empty. Not
// simply t − TAPE_SECONDS: a tape rewound by a reset or a parameter change
// holds only what has been run since.
const tapeStart = (tp) => (tp && tp.len
  ? tp.buf[(((tp.head - tp.len + TAPE_CAP) % TAPE_CAP) * 3) + 2]
  : Infinity);

// --- Loop ------------------------------------------------------------------

function recordTrail() {
  const cutoff = simTime - trailSeconds;

  // Recorded whether or not the checkbox is set, so switching a trail back on
  // shows the path it would have drawn. Starts at the first bob the chain
  // offers a box for, so a double's bob 0 costs nothing.
  for (const c of world) {
    const p = positions(c);
    for (let i = firstTraced(c); i < c.n; i++) {
      const pts = c.trails[i];
      pts.push([p[i][0], p[i][1], c.t]);
      while (pts.length && pts[0][2] < cutoff) pts.shift();

      const tp = c.tape[i] || (c.tape[i] = makeTape());
      if (c.t >= tp.next) tapePush(tp, p[i][0], p[i][1], c.t);
    }
  }
}

function frame(now) {
  // Measured against performance.now() rather than the rAF timestamp, which is
  // when the frame was scheduled, not when this function started.
  const t0 = performance.now();

  if (running) {
    // Unconsumed time carries into the next frame, so the simulation keeps pace
    // with the wall clock. Every chain banks the same real time.
    const dt = Math.min((now - lastFrame) / 1000, MAX_FRAME);
    lastFrame = now;
    for (const c of world) c.pending += dt;

    // The flight clock runs on simulated time, so pausing holds the profile
    // where it is. Gravity is set once per frame, not per sub-step: at the
    // steepest transition that is under 0.09 m/s² of lag, and only in the ramps.
    if (flightOn) applyFlight();

    // Clamped phase: the clock runs and the aircraft flies the profile while the
    // pieces are held at their initial conditions. Only the time up to the
    // release is consumed this way, so the release lands on its set time rather
    // than up to a frame late.
    const held = flightOn && flightTime < release;
    if (held) {
      const hold = Math.min(world[0].pending, release - flightTime);
      for (const c of world) {
        c.pending -= hold;
        c.t += hold;
      }
      simTime += hold;
      flightTime += hold;
    }

    const before = simTime;
    advance();
    // One clock for the world, taken from the chain furthest along.
    for (const c of world) if (c.t > simTime) simTime = c.t;

    if (flightOn) flightTime += simTime - before;
    if (!held) recordTrail();
  }

  draw();
  drawProfile();
  updateReadout();
  if (flightOn) updateFlightReadout();

  // Last, so it measures the whole frame, readouts included.
  recordFrame(now, performance.now() - t0);
  requestAnimationFrame(frame);
}

// --- Controls --------------------------------------------------------------

const el = (id) => document.getElementById(id);

const clamp = (v, lo, hi) => Math.min(Math.max(v, lo), hi);

function num(id, fallback) {
  const v = parseFloat(el(id).value);
  return Number.isFinite(v) ? v : fallback;
}

// One tab, tab dot, energy row, trail row and trail checkbox per slot, looked up
// once — the energy cells especially, since they are rewritten every frame.
const TABS = [];
const DOTS = [];
const E_ROW = [];
const E_DOT = [];
const E_CELL = [];
const TRAIL_ROW = [];
const TRAIL_BOX = [];
for (let p = 0; p < MAX_PENDULUMS; p++) {
  TABS.push(el('tab-' + (p + 1)));
  DOTS.push(TABS[p].querySelector('.dot'));
  E_ROW.push(el('e-row-' + (p + 1)));
  E_DOT.push(E_ROW[p].querySelector('.dot'));
  E_CELL.push(['e-pe-', 'e-ke-', 'e-total-'].map((id) => el(id + (p + 1))));
  TRAIL_ROW.push(el('trail-row-' + (p + 1)));
  TRAIL_BOX.push([0, 1, 2].map((i) => el('trail-' + (p + 1) + '-' + (i + 1))));
}

// Whether bob i of chain c is being recorded and drawn.
const traced = (c, i) => TRAIL_BOX[c.slot][i].checked;

// The first bob of this chain worth tracing: the second, except on a single,
// where it is the only bob there is. Bob 0 of a longer chain hangs on a fixed
// rod from a fixed pivot, so its path is the circle that rod already sweeps.
const firstTraced = (c) => (c.n === 1 ? 0 : 1);

// Trace this chain's tip and nothing else, so the default worst case is three
// trails rather than nine.
function tipOnly(c) {
  for (let i = 0; i < MAX_LINKS; i++) TRAIL_BOX[c.slot][i].checked = i === c.n - 1;
}

// Keeps a range slider and a number box showing the same value, and calls
// onChange whenever either moves.
// The bounds are read on every use rather than cached, so a control whose
// range moves at runtime (release, when the cycle is shortened) still clamps.
function linkPair(id, onChange) {
  const box = el(id);
  const slider = el(id + '-range');

  const push = (source) => {
    const raw = parseFloat(source.value);
    const v = clamp(raw, parseFloat(box.min), parseFloat(box.max));
    if (!Number.isFinite(v)) return;
    slider.value = v;
    // Write back when the slider moved it or clamping did, so the box never
    // shows a number the simulation is not using. Mid-typing values inside the
    // range are left alone.
    if (source !== box || v !== raw) box.value = v;
    onChange(v);
  };

  slider.addEventListener('input', () => push(slider));
  box.addEventListener('input', () => push(box));
  return (v) => { box.value = v; push(box); };
}

// Rod lengths, masses and friction take effect immediately, mid-flight: angles
// and rates carry over and the motion continues under the new physics. Every
// link is wired, including a hidden third. They write into chains[sel], which is
// why selectPendulum assigns sel before repopulating them — see the note there.
const setL = [];
const setM = [];
for (let i = 0; i < MAX_LINKS; i++) {
  setL.push(linkPair('L' + (i + 1), (v) => { chains[sel].L[i] = v; }));
  setM.push(linkPair('m' + (i + 1), (v) => { chains[sel].m[i] = v; }));
}
const setB = linkPair('b', (v) => { chains[sel].b = v; });
// Gravity is the aircraft's, so it stays outside the selection.
const setGravity = linkPair('g', (v) => { env.g = v; });
linkPair('trail-len', (v) => { trailSeconds = v; });

// Per chain too, so they are read as they are typed rather than only at reset;
// otherwise switching tabs would have to harvest them first.
function bindInit(id, write) {
  el(id).addEventListener('input', () => {
    const v = parseFloat(el(id).value);
    if (Number.isFinite(v)) write(chains[sel], v);
  });
}

for (let i = 0; i < MAX_LINKS; i++) {
  bindInit('t' + (i + 1), (c, v) => { c.th0[i] = v; });
  bindInit('w' + (i + 1), (c, v) => { c.om0[i] = v; });
}

// Link count is architecture, not a live parameter: changing it re-runs from the
// initial conditions rather than growing a limb mid-swing. Zero links is off —
// whether a pendulum hangs and how many rods it hangs by are one control,
// because a pendulum with no rods is a pendulum that is not there.
const linkButtons = {
  0: el('links-0'), 1: el('links-1'), 2: el('links-2'), 3: el('links-3')
};

// What the switch reads for a chain: its link count while hanging, 0 when not.
// The chain keeps its n either way, so the panel shows the right rows for a
// pendulum that is switched off.
const linksOf = (c) => (hanging(c) ? c.n : 0);

function setLinks(count) {
  const c = chains[sel];
  if (count === 0) {
    // Guarded as well as disabled: an empty world would take the stage scale,
    // the clock and the clamped phase with it.
    if (world.length < 2) return;
    world.splice(world.indexOf(c), 1);
  } else {
    c.n = count;
    c.s = new Float64Array(2 * count);
    // The chain has a new tip, and the trail was following the old one.
    tipOnly(c);
    if (!hanging(c)) {
      world.push(c);
      world.sort((a, x) => a.slot - x.slot);
    }
  }
  paintSelection();
  reset();
}

for (const [k, button] of Object.entries(linkButtons)) {
  const count = Number(k);
  button.addEventListener('click', () => {
    if (count !== linksOf(chains[sel])) confirmReset(() => setLinks(count));
  });
}

// --- Selection --------------------------------------------------------------

// Sets which pendulum the panel is editing, then repopulates every per-chain
// control from it. Any slot can be selected, hanging or not.
//
// sel is assigned first, and must be: the setters below call linkPair's push(),
// which fires onChange, which writes to chains[sel]. Repopulating before
// switching would write tab B's values into pendulum A on the way past.
function selectPendulum(slot) {
  sel = slot;
  const c = chains[slot];

  for (let k = 0; k < MAX_LINKS; k++) {
    setL[k](c.L[k]);
    setM[k](c.m[k]);
    el('t' + (k + 1)).value = c.th0[k];
    el('w' + (k + 1)).value = c.om0[k];
  }
  setB(c.b);

  paintSelection();
}

// Everything on the panel that says which pendulum is selected, and which of
// them are hanging. Split out from selectPendulum because a language switch and
// a change of link count need it without touching the values.
function paintSelection() {
  const c = chains[sel];

  for (const [k, button] of Object.entries(linkButtons)) {
    button.classList.toggle('on', Number(k) === linksOf(c));
  }
  // Something has to be hanging, so the last pendulum on the pivot cannot be
  // switched off. Disabled rather than hidden, so the segment keeps its place.
  linkButtons[0].disabled = hanging(c) && world.length < 2;

  // A link's controls and readouts appear and disappear together, so the panel
  // shows only the links this pendulum has. Keyed off c.n rather than the
  // switch, so one that is switched off still shows the shape it will have.
  for (const id of ['row-L2', 'row-m2', 'init-t2', 'init-w2', 'r-row-t2', 'r-row-w2']) {
    el(id).hidden = c.n < 2;
  }
  for (const id of ['row-L3', 'row-m3', 'init-t3', 'init-w3', 'r-row-t3', 'r-row-w3']) {
    el(id).hidden = c.n < 3;
  }

  for (let p = 0; p < MAX_PENDULUMS; p++) {
    const chain = chains[p];
    const on = hanging(chain);
    // The tab says which pendulum the panel is on; its dot says whether that
    // pendulum is on the pivot. Two marks, since a tab can be selected and empty.
    TABS[p].classList.toggle('on', p === sel);
    TABS[p].title = fmt(on ? 'arch.pendulum' : 'arch.pendulum.off', { n: CHAIN_NAME[p] });
    DOTS[p].classList.toggle('off', !on);
    // The same mark on this pendulum's energy row, which is all that names the
    // row. Selection is not marked there: Energy shows all three either way.
    E_DOT[p].classList.toggle('off', !on);
    E_ROW[p].classList.toggle('off', !on);
    TRAIL_ROW[p].hidden = !on;
    // One toggle per link the chain has, except that bob 1 is offered only on a
    // single. They carry a bare numeral, so the tooltip names the bob and the
    // pendulum.
    for (let i = 0; i < MAX_LINKS; i++) {
      const tog = el('trail-' + (p + 1) + '-' + (i + 1) + '-row');
      tog.hidden = !on || (i === 0 ? chain.n !== 1 : chain.n < i + 1);
      tog.title = fmt('trails.trace', { i: i + 1, n: CHAIN_NAME[p] });
    }
  }

  set('legend-state', fmt('state.legend', { n: chainName(c) }));
  // Hollow when the selected pendulum is not hanging. The Energy box has no
  // legend dot — it is not on one pendulum, and its rows carry their own.
  el('dot-state').className = 'dot p' + (c.slot + 1) + (hanging(c) ? '' : ' off');
}

// Selecting a tab resets nothing: it changes which pendulum the panel is
// pointed at, and the run carries on underneath. Switching one on or off does
// reset the world, so that every chain shares one t = 0.
for (let p = 0; p < MAX_PENDULUMS; p++) {
  TABS[p].addEventListener('click', () => {
    if (p !== sel) selectPendulum(p);
  });
}

// The two style buttons are one control. Points already recorded are redrawn in
// the new style, so switching clears nothing.
const trailStyles = { dots: el('trail-dots'), line: el('trail-line') };
for (const [style, button] of Object.entries(trailStyles)) {
  button.addEventListener('click', () => {
    trailStyle = style;
    for (const [key, b] of Object.entries(trailStyles)) b.classList.toggle('on', key === style);
  });
}

for (const b of document.querySelectorAll('.presets button')) {
  b.addEventListener('click', () => setGravity(parseFloat(b.dataset.g)));
}

// --- Flight mode -----------------------------------------------------------

// The profile owns gravity while it runs, so the manual g controls lock. Every
// other parameter stays live, and can be changed mid-parabola.
const gControls = [el('g'), el('g-range'), ...document.querySelectorAll('.presets button')];

// One number, written once a frame; how many chains read it is not its business.
function applyFlight() {
  env.g = flightAt(flightTime).level * G_EARTH;
  el('g').value = env.g.toFixed(2);
  el('g-range').value = env.g;
}

function updateFlightReadout() {
  const { u, ph, level } = flightAt(flightTime);
  const held = flightOn && flightTime < release;
  set('f-phase', held ? phaseName(ph) + ' · ' + t('flight.clamped') : phaseName(ph));
  set('f-g', level.toFixed(2) + ' g');
  set('f-para', fmt('flight.parabola', { n: Math.floor(flightTime / cycle()) + 1 }));
  set('f-t', fmt('flight.clock', { u: u.toFixed(1), c: cycle() }));
}

// Where the run starts and which phase the release lands in. Assembled from the
// template, since the clauses reorder between languages; values go in as text
// nodes, never as markup.
//
// Takes the value rather than reading `release`, because while the slider is
// dragged the note previews a release that has not been committed — the run is
// still flying the old one. See setRelease below.
function updateReleaseNote(v = release) {
  const vals = {
    '{start}': ['r-start', `t = ${Math.max(0, v - LEAD_IN)} s`],
    '{when}': ['r-when', `t = ${v} s`],
    '{where}': ['r-where', phaseInline(flightAt(v).ph)]
  };

  const p = el('release-note');
  p.textContent = '';
  for (const part of t('release.note').split(/(\{\w+\})/)) {
    if (!part) continue;
    if (vals[part]) {
      const span = document.createElement('span');
      [span.id, span.textContent] = vals[part];
      p.appendChild(span);
    } else {
      p.appendChild(document.createTextNode(part));
    }
  }
}

// Repaints every translated string wholesale, so no label can be left behind in
// the old language.
function applyLang() {
  document.documentElement.lang = lang;
  document.title = t('page.title');

  for (const node of document.querySelectorAll('[data-i18n]')) node.textContent = t(node.dataset.i18n);
  for (const node of document.querySelectorAll('[data-i18n-title]')) node.title = t(node.dataset.i18nTitle);
  // For a control whose name is not written on the panel — the links switch,
  // whose label the tabs above it took over.
  for (const node of document.querySelectorAll('[data-i18n-aria]')) {
    node.setAttribute('aria-label', t(node.dataset.i18nAria));
  }

  // The switch shows the language it would take you to, not the one you are in.
  const other = lang === 'fr' ? 'en' : 'fr';
  const btn = el('lang');
  btn.textContent = other.toUpperCase();
  btn.title = LANG_TIP[other];
  btn.setAttribute('aria-label', LANG_TIP[other]);

  // The rest is written by JS, so it has to be asked to redraw. The State legend
  // and the tab tooltips carry the pendulum's letter, so they come from the
  // selection rather than from a bare key.
  el('play').textContent = t(running ? 'btn.pause' : 'btn.start');
  paintSelection();
  updateReleaseNote();
  updateFlightReadout();
}

function setFlight(on) {
  flightOn = on;
  flightTime = 0;
  el('flight-on').checked = on;
  el('flight').classList.toggle('on', on);
  el('g-driven').hidden = !on;
  for (const c of gControls) c.disabled = on;

  if (on) {
    // Start from the initial conditions, so a chosen release time clamps the
    // pieces where it should. Leaves the world paused, as every reset does.
    reset();
  } else {
    // Hand gravity back at whatever value the profile had reached.
    setGravity(Math.round(env.g * 100) / 100);
    updateFlightReadout(); // clear the display rather than leave it stale
    // Switching off does not reset, so this is the only announcement there is,
    // and with no profile there are no cycle times to export from.
    worldChanged();
  }
}

// Switching off hands gravity back where it stands and lets the run carry on,
// so only switching on has anything to ask about.
el('flight-on').addEventListener('change', (e) => {
  if (!e.target.checked) {
    setFlight(false);
    return;
  }
  confirmReset(() => setFlight(true), () => { e.target.checked = false; });
});

// Release time defines the experiment, so changing it re-runs from the start,
// and the re-run waits for the drag to finish — change rather than input, since
// the slider crosses 180 values on the way to the one that was wanted.
//
// `release` is therefore not written while the slider moves: it is the release
// the run is flying, and the control holds the pending one until committed. The
// clamp test reads `release` every frame and startTime() is the origin export.js
// measures its windows from, so a value changing under a running flight would
// freeze the piece mid-swing and shift every recorded timestamp. The note
// follows the slider, as a preview.
const setRelease = linkPair('release', updateReleaseNote);

function commitRelease() {
  const v = num('release', release);
  if (v === release) return;
  // Off the profile nothing is flying it, so it waits for the next run.
  if (!flightOn) {
    release = v;
    updateReleaseNote();
    return;
  }
  confirmReset(
    () => { release = v; reset(); updateReleaseNote(); },
    () => { setRelease(release); } // put the control back where the run left it
  );
}

el('release').addEventListener('change', commitRelease);
el('release-range').addEventListener('change', commitRelease);

// Shortening the cycle can strand a release time past the end of it, so the
// control's range follows and the current value is pulled back inside.
function applyShortcut(on) {
  shortcut = on;
  const lim = cycle();
  el('release').max = lim;
  el('release-range').max = lim;
  if (release > lim) {
    release = lim;
    setRelease(lim); // the control follows, since nothing else moves it here
  }
  updateReleaseNote();
  if (flightOn) reset();
  // Off the profile there is no reset to carry the news, and the cycle has
  // still changed length under anything reading it.
  worldChanged();
}

el('shortcut').addEventListener('change', (e) => {
  // Off the profile the cycle length drives nothing running, so it asks nothing.
  if (!flightOn) {
    applyShortcut(e.target.checked);
    return;
  }
  confirmReset(() => applyShortcut(e.target.checked), () => { e.target.checked = shortcut; });
});

// --- Telling the page above ------------------------------------------------

// Anything built on top of the simulation registers here to be told the world
// has been re-run, cleared, or had the shape of its cycle changed. A one-way
// announcement: index.js never looks inside the list, and a page without
// export.js leaves it empty. Needed because a change may be waiting on the
// dialog below rather than landing on the same turn as the click.
const worldHooks = [];
const worldChanged = () => { for (const fn of worldHooks) fn(); };

// --- Asking ----------------------------------------------------------------

// The one question this page asks, in a <dialog>. showModal() handles centring,
// the backdrop, the focus trap and Escape, so none of that is written here.
//
// The answer arrives asynchronously, so callers hand over what to do rather than
// branching on a return value.
const askBox = el('ask');
let askYes = null;
let askNo = null;
let askWasRunning = false;

function ask(title, body, onYes, onNo) {
  set('ask-title', title);
  set('ask-text', body);
  askYes = onYes;
  askNo = onNo;
  // Frozen while the question is on screen, and put back if the answer is no:
  // the run being asked about should not be seconds further on by the time the
  // answer comes.
  askWasRunning = running;
  if (running) setRunning(false);
  askBox.returnValue = '';
  askBox.showModal();
  el('ask-yes').focus();
}

// One place where the answer is read, so Escape, the backdrop and the Cancel
// button are all the same no. showModal sets returnValue from close(v); Escape
// leaves it empty, which is what makes no the default.
askBox.addEventListener('close', () => {
  const yes = askBox.returnValue === 'yes';
  const fn = yes ? askYes : askNo;
  askYes = null;
  askNo = null;
  // The yes branch re-runs the world and reset() pauses, so only no has a run to
  // hand back. Restored before the callback, so a callback that starts something
  // cannot be undone by this.
  if (!yes && askWasRunning) setRunning(true);
  if (fn) fn();
});

el('ask-yes').addEventListener('click', () => askBox.close('yes'));
el('ask-no').addEventListener('click', () => askBox.close('no'));

// Link count, whether a pendulum hangs, the release time and the cycle length
// cannot be applied to a run in progress: changing one re-runs the world and
// takes the trails with it, so it asks first.
//
// With nothing to lose it does not ask, and runs onYes on this turn rather than
// a later one, so a caller can rely on the change having happened by the time
// its own handler returns.
function confirmReset(onYes, onNo) {
  if (simTime === 0) {
    onYes();
    return;
  }
  ask(t('reset.title'), t('reset.confirm'), onYes, onNo);
}

// Every chain from its own initial conditions, and the clock once: one pivot,
// one t = 0. Chains that are not hanging are reset too, so a tab shows the
// pendulum it will put on the pivot rather than whatever it was left holding.
function reset() {
  if (flightOn) {
    flightTime = startTime();
    applyFlight();
    updateFlightReadout();
  }
  for (const c of chains) {
    for (let i = 0; i < c.n; i++) {
      c.s[i] = (c.th0[i] * Math.PI) / 180;
      c.s[c.n + i] = (c.om0[i] * Math.PI) / 180;
    }
    c.pending = 0;
    c.t = 0;
  }
  simTime = 0;
  clearTrails();
  // Paused, always: the profile runs to 180 s, so which second you are watching
  // is the experiment, and the run starts on Start.
  setRunning(false);
  worldChanged();
}

function clearTrails() {
  for (const c of chains) {
    c.trails = [[], [], []];
    // Rewound rather than dropped: half a megabyte a bob, and this runs on every
    // parameter change, not only on Clear.
    for (const tp of c.tape) if (tp) { tp.head = 0; tp.len = 0; tp.next = 0; }
  }
}

function setRunning(on) {
  running = on;
  lastFrame = performance.now();
  for (const c of chains) c.pending = 0;
  el('play').textContent = t(on ? 'btn.pause' : 'btn.start');
}

el('play').addEventListener('click', () => setRunning(!running));

el('lang').addEventListener('click', () => {
  lang = lang === 'fr' ? 'en' : 'fr';
  applyLang();
});

// No confirmation: the button says Reset. It pauses, because reset() does.
el('reset').addEventListener('click', reset);

// The announcement is here rather than inside clearTrails, which reset() already
// calls on its own way to announcing itself.
el('clear').addEventListener('click', () => { clearTrails(); worldChanged(); });

// Snapshot the selected chain's live state into its initial-condition boxes, so
// an interesting configuration found mid-flight can be replayed.
el('use-current').addEventListener('click', () => {
  const c = chains[sel];
  for (let i = 0; i < c.n; i++) {
    c.th0[i] = Number(wrapDeg(c.s[i]).toFixed(1));
    c.om0[i] = Number(deg(c.s[c.n + i]).toFixed(1));
    el('t' + (i + 1)).value = c.th0[i];
    el('w' + (i + 1)).value = c.om0[i];
  }
});

// Space toggles run/pause, unless a text box has focus, or the confirmation is
// up — there Space belongs to the focused button.
document.addEventListener('keydown', (e) => {
  if (askBox.open) return;
  if (e.code === 'Space' && e.target.tagName !== 'INPUT') {
    e.preventDefault();
    setRunning(!running);
  }
});

// Pendulum A takes its parameters from the boxes rather than from DEFAULTS, so
// the markup and the load cannot disagree.
for (let i = 0; i < MAX_LINKS; i++) {
  chains[0].L[i] = num('L' + (i + 1), DEFAULTS[0].L[i]);
  chains[0].m[i] = num('m' + (i + 1), DEFAULTS[0].m[i]);
  chains[0].th0[i] = num('t' + (i + 1), i === 0 ? 90 : 0);
  chains[0].om0[i] = num('w' + (i + 1), 0);
}
chains[0].b = num('b', 0.001);

// B and C come fully built out of DEFAULTS at load, not at switch-on, so tuning
// one before it hangs is not overwritten when it does. Only their trails are
// left, since a checkbox cannot be derived from a link count in the markup.
for (let p = 1; p < MAX_PENDULUMS; p++) tipOnly(chains[p]);
env.g = num('g', 9.81);
release = num('release', 0);
trailSeconds = num('trail-len', 25);
applyLang(); // also paints the selection, the release note and the flight readout
sizeCanvas();
reset();
// Through the same path a click takes, so the default is one attribute in the
// markup rather than a second copy of the wiring.
setFlight(el('flight-on').checked);
requestAnimationFrame((now) => {
  lastFrame = now;
  frame(now);
});
