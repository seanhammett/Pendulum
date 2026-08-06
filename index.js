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

// Pendulums the page has: three rows, three colours, three trail rows, all
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
// slot is the chain's identity — colour, row, trail row and readout all key off
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
    // Each traced bob's path as world-space [x, y, t, g], indexed by bob, so a
    // trail survives a change of view scale. g is the gravity the point was
    // recorded under, kept whichever colour mode is on, so switching to the
    // gravity ramp colours the points already drawn rather than starting again.
    // Entry 0 is filled only on a single pendulum; on a longer chain that bob's
    // circle is its own rod's sweep.
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
//
// All three hang at load, so the page opens on the comparison it exists to
// make: one length of rod split one, two and three ways, released together.
const world = [chains[0], chains[1], chains[2]];
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

// Dark by default, and not persisted either, for the same reason: the page
// opens the same way every time. 'dark' is the bare :root palette, so the
// attribute is only ever set to switch away from it.
let theme = 'dark';

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
// Re-read rather than read once: the theme switch rewrites every one of these
// properties, and the canvas has to follow it. readPalette() overwrites the
// same object in place, so everything that closed over COLOUR keeps working.
const css = getComputedStyle(document.documentElement);
const prop = (name) => css.getPropertyValue(name).trim();
const COLOUR = {};

function readPalette() {
  Object.assign(COLOUR, {
    // The pivot belongs to the world rather than any one chain, so it is
    // neutral.
    pivot: prop('--text'),
    // Rods are one colour for every chain; the chain's colour runs down their
    // core.
    rod: prop('--rod'),
    // Panel chrome, used by the flight profile.
    accent: prop('--accent'),
    // The inset a number box sits on, which is what an off pendulum's bobs are
    // filled with in the Architecture figures.
    line: prop('--line'),
    // Nothing is driving it: the profile curve with the flight simulator
    // switched off, a pendulum that is off the pivot. Grey rather than a
    // dimmed chain colour, so "not running" is the same mark wherever it
    // appears.
    inert: prop('--inert'),
    // The shading over the profile's clamped span, and the release marker
    // standing at its edge.
    shade: prop('--shade'),
    muted: prop('--muted'),
    // Indexed by slot, not by position in the world.
    chain: [0, 1, 2].map((p) => prop('--p' + (p + 1))),
    // The two ends of the gravity ramp, mixed into GRAV_RAMP below.
    grav: [prop('--grav-0'), prop('--grav-max')]
  });
  buildRamp();
}

// --- The gravity ramp ------------------------------------------------------
//
// The colour a trail is drawn in while the gravity mode is on: one colour for
// every chain, taken from the magnitude of g at the moment each point was
// recorded rather than from which pendulum recorded it.
//
// Mixed once into a table of shades rather than per point per frame, and
// quantised, which is also what lets drawTrail batch a run of points into one
// draw — see the second cut it makes there.
const TRAIL_SHADES = 48;
const GRAV_RAMP = [];

// #rgb and #rrggbb both, since these two ends are written by hand in the
// stylesheet.
function hexRGB(hex) {
  const s = hex.replace('#', '');
  const w = s.length === 3 ? s.replace(/./g, (c) => c + c) : s;
  return [0, 2, 4].map((i) => parseInt(w.slice(i, i + 2), 16));
}

function buildRamp() {
  const [lo, hi] = COLOUR.grav.map(hexRGB);
  GRAV_RAMP.length = 0;
  for (let i = 0; i < TRAIL_SHADES; i++) {
    const x = i / (TRAIL_SHADES - 1);
    const [r, g, b] = lo.map((c, k) => Math.round(c + (hi[k] - c) * x));
    GRAV_RAMP.push(`rgb(${r},${g},${b})`);
  }
}

// The top of the ramp is the profile's own ceiling — the 1.8 g of the
// hypergravity phases — read off the phase table rather than written again, so
// the darkest a trail goes is the heaviest the aircraft flies.
const GRAV_TOP = G_EARTH * Math.max(...PHASES.map((p) => Math.max(p.from, p.to)));

// Magnitude, so an aircraft pushing down reads as the same weight as one pushing
// up, and clamped at both ends: the manual g control reaches well past the
// profile's ceiling in both directions, and everything beyond it is the darkest
// the ramp has.
const shadeOf = (g) => GRAV_RAMP[Math.round(
  Math.min(Math.max(Math.abs(g) / GRAV_TOP, 0), 1) * (TRAIL_SHADES - 1)
)];

readPalette();

// Where you are in the flight cycle, on the profile: the playhead, and the two
// bounds of the span an export will cover. Deliberately not --accent, which the
// curve and its fill already wear, and not one of the three chain colours.
const MARK = '#ffb347';

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

// And two things a trail can be coloured by:
//
//   'chain'   — the pendulum that drew it, so a chain is one colour wherever it
//               appears on the page. The default.
//   'gravity' — the gravity it was drawn under, one ramp for every chain, so
//               the trail carries the flight profile as well as the path.
//
// Both read the same recorded points, which carry the g they were recorded at:
// switching redraws what is already there rather than clearing it, exactly as
// the style pair does.
let trailColour = 'chain';

function drawTrail(pts, colour, toX, toY) {
  if (!pts.length) return;
  const dots = trailStyle === 'dots';
  const byG = trailColour === 'gravity';
  ctx.lineWidth = 1.5;
  ctx.lineJoin = 'round';
  // The chain's colour is set once for the whole trail; the gravity mode is the
  // one that has a new colour to set on every run.
  if (!byG) {
    ctx.fillStyle = colour;
    ctx.strokeStyle = colour;
  }

  // One batched draw of pts[a…b), in `shade` or in the colour already set, at
  // whatever alpha is set.
  const run = (a, b, shade) => {
    if (shade) {
      ctx.fillStyle = shade;
      ctx.strokeStyle = shade;
    }
    ctx.beginPath();
    for (let i = a; i < b; i++) {
      const x = toX(pts[i][0]);
      const y = toY(pts[i][1]);
      if (!dots) {
        if (i === a) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        continue;
      }
      // Start a fresh subpath at each dot, or the arcs are joined by the line
      // this style exists to do without.
      ctx.moveTo(x + TRAIL_DOT, y);
      ctx.arc(x, y, TRAIL_DOT, 0, Math.PI * 2);
    }
    if (dots) ctx.fill(); else ctx.stroke();
  };

  for (let c = 0; c < TRAIL_CHUNKS; c++) {
    const from = Math.floor((c * pts.length) / TRAIL_CHUNKS);
    const to = Math.floor(((c + 1) * pts.length) / TRAIL_CHUNKS);
    if (to <= from) continue;
    // The line starts each chunk one point early, so the segment spanning a
    // chunk boundary is drawn and the fade has no gaps. Dots need no overlap,
    // and repeating one would only darken it.
    const start = dots ? from : Math.max(0, from - 1);

    ctx.globalAlpha = ((c + 1) / TRAIL_CHUNKS) * 0.75;
    if (!byG) {
      run(start, to);
      continue;
    }

    // In the gravity mode the colour moves along the chunk as well as between
    // chunks, and faster: 1.8 g to 0 inside the 5 s injection, a twelfth of the
    // longest trail. So the chunk is cut again wherever the shade steps, which
    // holds the ramp smooth at any trail length — and costs nothing over the
    // fade's own 24 draws while g is flat, which is most of a cycle.
    let a = start;
    let shade = shadeOf(pts[a][3]);
    for (let i = a + 1; i < to; i++) {
      const next = shadeOf(pts[i][3]);
      if (next === shade) continue;
      // The line hands the shared point on to the next run, so the segment
      // across the step is drawn once, in the older shade.
      run(a, dots ? i : i + 1, shade);
      a = i;
      shade = next;
    }
    run(a, to, shade);
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

// --- The panel figures -----------------------------------------------------

// One inline svg per pendulum in the Architecture box. Straight and horizontal:
// this is what a pendulum is made of, not what it is doing, so it does not
// follow the angles. Every weight mirrors draw() above at panel scale, so a rod,
// a bob and a pivot read as the same objects in both places.
const FIG_H = 26; // row height, sized for the heaviest bob and its ring
const FIG_BOB = 6; // what bobR is on the canvas
const FIG_ROD = 2.5;
const FIG_RING = 1.5;
const FIG_PAD = 3; // the pivot's own margin at the left edge

// The same law the canvas uses: radius follows mass by volume, clamped.
const figR = (m) => FIG_BOB * clamp(Math.cbrt(m), 0.6, 1.7);

const SVG_NS = 'http://www.w3.org/2000/svg';

function svgEl(name, attrs) {
  const node = document.createElementNS(SVG_NS, name);
  for (const key of Object.keys(attrs)) node.setAttribute(key, attrs[key]);
  return node;
}

// Draws one chain into its row's svg. scale is pixels per metre, shared by the
// three rows and set by the longest of them, exactly as the canvas scales every
// chain to the largest reach in the world: a shorter pendulum draws shorter, so
// an edit to a total is visible.
//
// Redrawn on an edit and on a resize, not per frame.
function drawFigure(svg, c, on, scale) {
  const cy = FIG_H / 2;
  const colour = on ? COLOUR.chain[c.slot] : COLOUR.inert;
  const rod = on ? COLOUR.rod : COLOUR.inert;
  const line = { x1: FIG_PAD, y1: cy, x2: FIG_PAD, y2: cy };
  for (let i = 0; i < c.n; i++) line.x2 += c.L[i] * scale;

  svg.textContent = '';
  svg.appendChild(svgEl('line', {
    ...line, stroke: rod, 'stroke-width': FIG_ROD, 'stroke-linecap': 'round'
  }));
  // The coloured core down the white rod. An off pendulum carries no chain
  // colour at all — its pill still does, as a hollow dot — but it keeps the
  // core, in the panel's line colour, so the rod reads as the same object
  // greyed rather than as a different one.
  svg.appendChild(svgEl('line', {
    ...line,
    stroke: on ? colour : COLOUR.line,
    'stroke-width': Math.max(1, FIG_ROD * CORE_RATIO)
  }));

  let d = 0;
  for (let i = 0; i < c.n; i++) {
    d += c.L[i];
    svg.appendChild(svgEl('circle', {
      cx: FIG_PAD + d * scale, cy, r: figR(c.m[i]),
      fill: on ? colour : COLOUR.line, stroke: rod, 'stroke-width': FIG_RING
    }));
  }

  // Last and neutral, as on the canvas: the pivot belongs to the world.
  svg.appendChild(svgEl('circle', {
    cx: FIG_PAD, cy, r: FIG_BOB * 0.45, fill: on ? COLOUR.pivot : COLOUR.inert
  }));

  // What a drag takes hold of: one invisible target over each joint, wider than
  // the bob under it, because a 6px circle is not something to ask a pointer to
  // hit. Over the bobs rather than under them so the grip wins the pointer, and
  // invisible because the bob is what it looks like.
  //
  // Not on the tip — see setJoint — and none at all on a row that is off the
  // pivot, where a click anywhere is the switch that hangs the pendulum.
  if (!on) return;
  let g = 0;
  for (let i = 0; i < c.n - 1; i++) {
    g += c.L[i];
    const grip = svgEl('circle', {
      class: 'arch-grip', 'data-grip': i,
      cx: FIG_PAD + g * scale, cy, r: Math.max(figR(c.m[i]) + 3, 8)
    });
    // The svg is aria-hidden: the two length boxes are this edit's keyboard and
    // screen-reader half, and they say so in their own tooltips. This one names
    // the pair the drag moves for a pointer that has found the bob first.
    const name = svgEl('title', {});
    name.textContent = fmt('arch.slide', { i: i + 1, j: i + 2 });
    grip.appendChild(name);
    svg.appendChild(grip);
  }
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
  lctx.strokeStyle = COLOUR.line;
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
  lctx.strokeStyle = flightOn ? COLOUR.accent : COLOUR.inert;
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
    pctx.fillStyle = COLOUR.shade;
    pctx.fillRect(0, 0, rx, h);
    pctx.strokeStyle = COLOUR.muted;
    pctx.setLineDash([2, 2]);
    pctx.lineWidth = 1;
    pctx.beginPath();
    pctx.moveTo(rx, 0);
    pctx.lineTo(rx, h);
    pctx.stroke();
    pctx.setLineDash([]);
  }

  // The span an export will cover. Drawn over the clamped shading, since a span
  // chosen before the release is still the span saved.
  //
  // Marked in the same orange as the playhead, because both are about where you
  // are in the cycle rather than what the aircraft is doing — but dashed, and
  // over a fill faint enough to read the curve through, so the two bounds cannot
  // be taken for the one line that moves.
  if (markSpan) {
    const a = toXt(clamp(markSpan[0], 0, C));
    const b = toXt(clamp(markSpan[1], 0, C));
    if (b > a) {
      pctx.fillStyle = COLOUR.accent;
      pctx.globalAlpha = 0.09;
      pctx.fillRect(a, top - 2, b - a, bottom - top + 4);
      pctx.globalAlpha = 1;
      pctx.strokeStyle = MARK;
      pctx.lineWidth = 1;
      pctx.setLineDash([1, 3]);
      pctx.beginPath();
      // Half-pixel offsets, and inward, so both edges land on a pixel boundary
      // and a span at t = 0 keeps its left edge on the canvas.
      pctx.moveTo(a + 0.5, top - 2);
      pctx.lineTo(a + 0.5, bottom + 2);
      pctx.moveTo(b - 0.5, top - 2);
      pctx.lineTo(b - 0.5, bottom + 2);
      pctx.stroke();
      pctx.setLineDash([]);
    }
  }

  const { u, level } = flightAt(flightTime);
  const x = toXt(u);
  pctx.strokeStyle = MARK;
  pctx.lineWidth = 1;
  pctx.beginPath();
  pctx.moveTo(x, top - 2);
  pctx.lineTo(x, bottom + 2);
  pctx.stroke();

  pctx.fillStyle = MARK;
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

// One decimal while the figure is small enough to earn it, none above 100: the
// three of them share the width of a row, so a number that grew a digit would
// have to come from somewhere. A trailing unit on each, since there is no legend
// over them to say it once.
const joules = (v) => v.toFixed(v < 100 ? 1 : 0) + ' J';

// Every pendulum on the pivot, angles then speeds then energies, into its own
// row and its own lines. Nothing is written for a pendulum that is off: it is not
// being stepped, and neither its Architecture row nor its State lines are on the
// panel to write into.
function updateReadout() {
  for (const c of world) {
    const r = READ[c.slot];
    for (let i = 0; i < c.n; i++) {
      setCell(r[i].th, wrapDeg(c.s[i]).toFixed(1) + '°');
      setCell(r[i].om, deg(c.s[c.n + i]).toFixed(0) + '°/s');
    }

    // Split rather than summed: in flight the potential term drains to nothing
    // at 0g while the kinetic term carries on untouched.
    const pe = potential(c);
    const ke = kinetic(c);
    const cell = E_CELL[c.slot];
    setCell(cell[0], joules(pe));
    setCell(cell[1], joules(ke));
    setCell(cell[2], joules(pe + ke));
  }
  set('r-time', simTime.toFixed(2) + ' s');
}

// Up to twenty-seven figures a frame, so a cell whose text has not changed is
// not written.
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
      // env.g rather than the profile's level: the manual control is gravity
      // too, and it is the number the chain was actually integrated with this
      // frame either way.
      pts.push([p[i][0], p[i][1], c.t, env.g]);
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

function numOf(box, fallback) {
  const v = parseFloat(box.value);
  return Number.isFinite(v) ? v : fallback;
}

const num = (id, fallback) => numOf(el(id), fallback);

// Everything the panel holds three of, looked up once and indexed by slot: the
// letter switch and its dot, the three energy cells, the trail row and its
// checkboxes, and — one entry per link — the initial-condition boxes and the
// State lines. The energy cells and the State figures especially, since they are
// rewritten every frame.
const TABS = [];
const DOTS = [];
const E_CELL = [];
const TRAIL_ROW = [];
const TRAIL_BOX = [];
const INIT = [];
const READ = [];
for (let p = 0; p < MAX_PENDULUMS; p++) {
  const n = p + 1;
  const each = (make) => [1, 2, 3].map(make);
  TABS.push(el('tab-' + n));
  DOTS.push(TABS[p].querySelector('.dot'));
  E_CELL.push(['e-pe-', 'e-ke-', 'e-total-'].map((id) => el(id + n)));
  TRAIL_ROW.push(el('trail-row-' + n));
  TRAIL_BOX.push([0, 1, 2].map((i) => el('trail-' + n + '-' + (i + 1))));
  INIT.push(each((i) => ({
    line: el('init-line-' + n + '-' + i),
    th: el('t-' + n + '-' + i),
    om: el('w-' + n + '-' + i)
  })));
  READ.push(each((i) => ({
    line: el('r-line-' + n + '-' + i),
    th: el('r-t-' + n + '-' + i),
    om: el('r-w-' + n + '-' + i)
  })));
}

// The Architecture box's three rows: each pendulum's figure, its length, mass and
// total boxes with the cells they sit in, and its two step buttons. All three
// rows are on the panel at once, so all of this is looked up once here rather
// than followed from a selection.
const ARCH = [];
for (let p = 0; p < MAX_PENDULUMS; p++) {
  const n = p + 1;
  const each = (make) => [1, 2, 3].map(make);
  ARCH.push({
    row: el('arch-row-' + n),
    fig: el('arch-fig-' + n),
    L: each((i) => el('L-' + n + '-' + i)),
    m: each((i) => el('m-' + n + '-' + i)),
    total: el('LT-' + n),
    cellL: each((i) => el('cell-L-' + n + '-' + i)),
    cellM: each((i) => el('cell-m-' + n + '-' + i)),
    add: el('arch-add-' + n),
    del: el('arch-del-' + n)
  });
}

// The pixels per metre each row's figure was last drawn at, which is what turns
// a pointer position over one back into a length. Written by paintArch, since
// choosing the scale is its job.
const FIG_SCALE = ARCH.map(() => 0);

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

// Friction takes effect immediately, mid-flight, as lengths and masses do:
// angles and rates carry over and the motion continues under the new physics.
// It writes into chains[sel], which is why selectPendulum assigns sel before
// repopulating it — see the note there.
const setB = linkPair('b', (v) => { chains[sel].b = v; });
// Gravity is the aircraft's, so it stays outside the selection.
const setGravity = linkPair('g', (v) => { env.g = v; });
linkPair('trail-len', (v) => { trailSeconds = v; });

// Read as they are typed rather than only at reset. Every pendulum's boxes are
// on the panel at once, so each writes into its own chain and none of this
// follows the selection.
function bindInit(box, write) {
  box.addEventListener('input', () => {
    const v = parseFloat(box.value);
    if (Number.isFinite(v)) write(v);
  });
}

for (let p = 0; p < MAX_PENDULUMS; p++) {
  const c = chains[p];
  for (let i = 0; i < MAX_LINKS; i++) {
    bindInit(INIT[p][i].th, (v) => { c.th0[i] = v; });
    bindInit(INIT[p][i].om, (v) => { c.om0[i] = v; });
  }
}

// --- Rods -------------------------------------------------------------------

// A chain's links and its reach are one set of numbers seen two ways, and the
// box offers both, so every edit has to keep the other side true: changing the
// total scales the links together, and changing a link makes the others take up
// the slack with the total held. Nothing here rounds — the boxes round for
// display only, so nudging one of them repeatedly cannot walk the total.
const LMIN = 0.1; // shortest link (m)
const LMAX_T = 3; // longest chain, pivot to tip (m)
const MMIN = 0.1; // lightest bob (kg)
const MMAX = 5;

const reachOf = (c) => {
  let sum = 0;
  for (let i = 0; i < c.n; i++) sum += c.L[i];
  return sum;
};

const shortestOf = (c) => {
  let lo = c.L[0];
  for (let i = 1; i < c.n; i++) lo = Math.min(lo, c.L[i]);
  return lo;
};

// Every link scaled by the same factor, so the chain keeps its shape and only
// its reach changes. It cannot be shrunk so far that a link falls under LMIN:
// on a chain of equal links that floor is LMIN·n, and on a lopsided one it is
// whatever the shortest link can survive.
function setTotal(c, v) {
  const T = reachOf(c);
  const want = clamp(v, (T * LMIN) / shortestOf(c), LMAX_T);
  const k = want / T;
  for (let i = 0; i < c.n; i++) c.L[i] *= k;
  return want;
}

// One link changed with the total held: the others give up or take back the
// difference in proportion, so their ratios to one another do not move. The
// link's own ceiling is whatever leaves every other link at LMIN or above, which
// on a chain of equal links is T − LMIN·(n−1).
function setLink(c, i, v) {
  // The one link of a single is the total, which is why its box carries the
  // total's accent border as well, and why LT beside it holds the same number.
  if (c.n === 1) return setTotal(c, v);

  const T = reachOf(c);
  const rest = T - c.L[i];
  let lo = Infinity;
  for (let j = 0; j < c.n; j++) if (j !== i) lo = Math.min(lo, c.L[j]);

  const want = clamp(v, LMIN, T - (LMIN * rest) / lo);
  const k = (T - want) / rest;
  for (let j = 0; j < c.n; j++) if (j !== i) c.L[j] *= k;
  c.L[i] = want;
  return want;
}

// Pivot to bob i, in metres — where drawFigure puts that bob, and what a drag
// on it is moving.
function jointOf(c, i) {
  let d = 0;
  for (let j = 0; j <= i; j++) d += c.L[j];
  return d;
}

// One bob slid along the chain to d, measured from the pivot: the two links
// either side of it give and take the whole difference, so their sum is held
// and everything outside the pair — the reach, every other bob, the links below
// them — does not move at all. That is the edit a drag on the figure makes, and
// it is a narrower one than setLink's: a length typed into a box is taken from
// every other link in proportion, because a box has no neighbour to prefer.
//
// Only for the joints between links. The tip has no link below it to hand the
// difference to, so sliding it could only be a change of reach, which is the
// total's job.
function setJoint(c, i, d) {
  let above = 0;
  for (let j = 0; j < i; j++) above += c.L[j];
  const span = c.L[i] + c.L[i + 1];
  // The pair can be split anywhere that leaves both halves a link: span is two
  // links, so it is never under 2·LMIN and this is never an empty range.
  c.L[i] = clamp(d - above, LMIN, span - LMIN);
  c.L[i + 1] = span - c.L[i];
}

// Adding and removing a link both hold the chain's reach, which is what makes
// the three rows a comparison: a triple is one length of rod divided three ways
// rather than a single with two more rods on the end. It is also the rule the
// shipped defaults were built to.
function addLink(c) {
  const n = c.n;
  const T = reachOf(c);
  // Every link gives up the same fraction of itself and the new one takes what
  // they gave. A link already sitting on LMIN has nothing to give, so the
  // fraction is capped by the shortest of them and the chain grows by the
  // shortfall instead of letting a link vanish — three links at LMIN is 0.3 m
  // against a 3 m ceiling, so there is room for that.
  const keep = Math.max(n / (n + 1), Math.min(1, LMIN / shortestOf(c)));
  for (let i = 0; i < n; i++) c.L[i] *= keep;
  c.L[n] = Math.max(LMIN, T * (1 - keep));
  c.n = n + 1;
  // Its mass is whatever this chain was holding for that link, which is the
  // markup's value until someone changes it; see DEFAULTS.
}

// The tip comes off and its length goes back into the links above it, in
// proportion, so those keep their ratios and the reach is unchanged.
function removeLink(c) {
  const T = reachOf(c);
  const k = T / (T - c.L[c.n - 1]);
  c.n -= 1;
  for (let i = 0; i < c.n; i++) c.L[i] *= k;
}

// Link count is architecture, not a live parameter: changing it re-runs from the
// initial conditions rather than growing a limb mid-swing.
function setLinkCount(slot, n) {
  const c = chains[slot];
  if (n < 1 || n > MAX_LINKS || n === c.n) return;
  if (n > c.n) addLink(c); else removeLink(c);
  c.s = new Float64Array(2 * c.n);
  // The chain has a new tip, and the trail was following the old one.
  tipOnly(c);
  paintSelection();
  reset();
}

// --- On the pivot -----------------------------------------------------------

// Splices a chain in or out of the world. A pendulum keeps everything about it
// while it is off, so switching it back on returns the same pendulum.
function setActive(slot, on) {
  const c = chains[slot];
  if (on === hanging(c)) return;
  if (on) {
    world.push(c);
    world.sort((a, x) => a.slot - x.slot);
  } else {
    // Guarded as well as disabled: an empty world would take the stage scale,
    // the clock and the clamped phase with it.
    if (world.length < 2) return;
    world.splice(world.indexOf(c), 1);
    // The panel edits something that is on the pivot. b writes into the selected
    // chain, and an off row carries no lit surround, so leaving the selection on
    // the pendulum that has just gone grey would leave the slider editing a
    // pendulum nothing on the panel names.
    if (slot === sel) selectPendulum(world[0].slot);
  }
  paintSelection();
  reset();
}

// Putting a pendulum back on the pivot also points the box at it: the row that
// was clicked is the row the box is now editing.
function activate(slot) {
  confirmReset(() => { selectPendulum(slot); setActive(slot, true); });
}

// --- The pendulum rows ------------------------------------------------------

// Every box writes straight into its own chain and then the box is repainted,
// because one edit moves its neighbours: a length moves the other lengths, a
// total moves all of them.
//
// Never the box being typed in, though — half a number is a number, and a box
// that rewrote itself on every keystroke would turn the 0 of 0.5 into the
// minimum before the point was reached. It is squared up on the way out
// instead, which is also when the value is rounded to what the box can show, so
// the simulation runs on the full precision it was given until then.
function bindArchBox(input, apply) {
  input.addEventListener('input', () => {
    const raw = parseFloat(input.value);
    if (!Number.isFinite(raw)) return;
    apply(raw);
    paintArch(input);
  });
  input.addEventListener('change', () => paintArch());
}

// A bob dragged along its own figure, which is the same edit as typing into the
// two length boxes either side of it and is applied the same way: straight into
// the chain, live, with no reset. A rod changing length mid-swing is a change of
// physics, not of architecture — see setLinkCount for the ones that are.
//
// The pointer is captured by the svg and not by the grip: the figure is redrawn
// on every move, so the circle the drag started on is gone by the second one.
// The svg itself survives — drawFigure empties it rather than replacing it.
function bindFigureDrag(fig, c, slot) {
  let held = -1; // which joint has the pointer, or none
  let grab = 0;  // metres between the pointer and the bob when it was taken, so
                 // a bob grabbed off-centre stays off-centre instead of jumping

  // Where the pointer is along this figure, in metres from the pivot.
  const along = (e) => (e.clientX - fig.getBoundingClientRect().left - FIG_PAD) / FIG_SCALE[slot];

  fig.addEventListener('pointerdown', (e) => {
    const i = Number(e.target.dataset.grip);
    if (e.button !== 0 || !Number.isInteger(i)) return;
    held = i;
    grab = along(e) - jointOf(c, i);
    fig.setPointerCapture(e.pointerId);
    // No text selection dragged across the panel behind the bob. This also
    // takes the click the row would have used to select the pendulum — a
    // pointerdown that is prevented raises no mouse event and so no click — so
    // the drag does that itself, below. A grip is only ever on a hanging row,
    // so selecting is the whole of what that click would have done.
    e.preventDefault();
    if (slot !== sel) selectPendulum(slot);
    // The cursor is the row's for as long as the drag lasts, not the grip's:
    // once a joint is against its stop the pointer runs on past the target.
    fig.classList.add('sliding');
  });

  fig.addEventListener('pointermove', (e) => {
    if (held < 0) return;
    setJoint(c, held, along(e) - grab);
    // Every box, not the two that moved: the figure is drawn from the same
    // numbers, and paintArch is what redraws it.
    paintArch();
  });

  const drop = () => {
    held = -1;
    fig.classList.remove('sliding');
  };
  fig.addEventListener('pointerup', drop);
  fig.addEventListener('pointercancel', drop);
}

for (let p = 0; p < MAX_PENDULUMS; p++) {
  const c = chains[p];
  const a = ARCH[p];
  for (let i = 0; i < MAX_LINKS; i++) {
    bindArchBox(a.L[i], (v) => setLink(c, i, v));
    bindArchBox(a.m[i], (v) => { c.m[i] = clamp(v, MMIN, MMAX); return c.m[i]; });
  }
  bindArchBox(a.total, (v) => setTotal(c, v));
  bindFigureDrag(a.fig, c, p);

  a.add.addEventListener('click', () => confirmReset(() => setLinkCount(p, c.n + 1)));
  a.del.addEventListener('click', () => confirmReset(() => setLinkCount(p, c.n - 1)));

  // Clicking anywhere in an active row selects that pendulum; an off row has no
  // controls in it, and clicking it is what puts the pendulum back on the pivot.
  a.row.addEventListener('click', () => {
    if (!hanging(c)) activate(p);
    else if (p !== sel) selectPendulum(p);
  });
  // Reaching a box by keyboard is the same choice as reaching it by pointer.
  a.row.addEventListener('focusin', () => { if (p !== sel) selectPendulum(p); });
}

// The figures are drawn in CSS pixels rather than to a viewBox, so a panel that
// changes width — the narrow layout, or a window dragged under it — has to have
// them drawn again. Repainting does not change the row's size, so this cannot
// feed itself.
new ResizeObserver(() => paintArch()).observe(ARCH[0].row);

// Everything inside the three rows: which boxes are there, what they say, the
// state of the pill and the figure beside it. Called after every edit, and by
// paintSelection, so a row's lit surround and its contents are never out of step.
//
// `typing` is the one box not to rewrite: the one the new value came from, which
// still has the cursor in it.
function paintArch(typing) {
  // One scale for the three rows, from the longest chain in the box whether it
  // is hanging or not — as the canvas scales every chain to the largest reach.
  let longest = 0;
  let widest = 0;
  for (const c of chains) {
    longest = Math.max(longest, reachOf(c));
    for (let i = 0; i < c.n; i++) widest = Math.max(widest, figR(c.m[i]));
  }

  for (let p = 0; p < MAX_PENDULUMS; p++) {
    const c = chains[p];
    const a = ARCH[p];
    const on = hanging(c);

    a.row.classList.toggle('off', !on);
    a.row.classList.toggle('sel', on && p === sel);
    // The off row is a switch, and the only one on the panel with no label.
    a.row.title = on ? '' : fmt('arch.pendulum.off', { n: CHAIN_NAME[p] });

    // The width the svg is given, less the head column, for the first paint of
    // a panel that has not been laid out yet.
    const width = a.fig.clientWidth || 207;
    FIG_SCALE[p] = (width - FIG_PAD - widest - 1) / longest;
    drawFigure(a.fig, c, on, FIG_SCALE[p]);

    for (let i = 0; i < MAX_LINKS; i++) {
      a.cellL[i].hidden = i >= c.n;
      a.cellM[i].hidden = i >= c.n;
      if (a.L[i] !== typing) a.L[i].value = c.L[i].toFixed(3);
      if (a.m[i] !== typing) a.m[i].value = c.m[i].toFixed(2);
    }

    // On a single the one link is the total, so both boxes carry the mark and
    // both scale the chain. The total keeps its place either way, which is what
    // holds the three rows in line.
    a.cellL[0].classList.toggle('total', c.n === 1);
    if (a.total !== typing) a.total.value = reachOf(c).toFixed(3);

    // The two round buttons carry a sign, not a name, so what they will do is
    // said in the tooltip: each names the link it adds or takes away. They grey
    // out where they run out rather than disappearing — add while the chain is
    // short of three links, remove while it has more than one.
    setStep(a.add, c.n >= MAX_LINKS, 'arch.add', c.n + 1);
    setStep(a.del, c.n < 2, 'arch.remove', c.n);
  }
}

// A step button's state and the name it answers to, which is the only place it
// is written: the face of the button is a + or a −.
function setStep(button, off, key, n) {
  const name = fmt(key, { n });
  button.disabled = off;
  button.title = name;
  button.setAttribute('aria-label', name);
}

// --- Selection --------------------------------------------------------------

// Sets which pendulum the panel is editing, then repopulates the one control
// that still follows the selection: friction. Nothing else does any more — the
// rods, the initial conditions and the state of every pendulum are all on the
// panel at once, each in its own row or line, always live.
//
// sel is assigned first, and must be: setB calls linkPair's push(), which fires
// onChange, which writes to chains[sel]. Repopulating before switching would
// write pendulum B's friction into pendulum A on the way past.
function selectPendulum(slot) {
  sel = slot;
  setB(chains[slot].b);
  paintSelection();
}

// Everything on the panel that says which pendulum is selected, and which of
// them are hanging. Split out from selectPendulum because a language switch and
// a change of link count need it without touching the values.
function paintSelection() {
  for (let p = 0; p < MAX_PENDULUMS; p++) {
    const chain = chains[p];
    const on = hanging(chain);
    // The letter is the switch, and its row lights when the pendulum is on the
    // pivot; the dot says the same thing a second way. Which of them the box is
    // editing is said by the same surround at full accent.
    //
    // Something has to be hanging, so the last one there cannot be switched off:
    // its letter disables rather than doing nothing when it is pressed.
    const last = on && world.length < 2;
    const key = last ? 'arch.pendulum.last' : (on ? 'arch.pendulum' : 'arch.pendulum.off');
    TABS[p].disabled = last;
    TABS[p].title = fmt(key, { n: CHAIN_NAME[p] });
    DOTS[p].classList.toggle('off', !on);
    TRAIL_ROW[p].hidden = !on;

    // One line each in Initial conditions and State per link this pendulum has,
    // and none at all while it is off the pivot: it is not being released from
    // anything and not being stepped. The two boxes hold the same shape, so one
    // condition covers both.
    for (let i = 0; i < MAX_LINKS; i++) {
      const show = on && i < chain.n;
      INIT[p][i].line.hidden = !show;
      READ[p][i].line.hidden = !show;
    }
    // One toggle per link the chain has, except that bob 1 is offered only on a
    // single. They carry a bare numeral, so the tooltip names the bob and the
    // pendulum.
    for (let i = 0; i < MAX_LINKS; i++) {
      const tog = el('trail-' + (p + 1) + '-' + (i + 1) + '-row');
      tog.hidden = !on || (i === 0 ? chain.n !== 1 : chain.n < i + 1);
      tog.title = fmt('trails.trace', { i: i + 1, n: CHAIN_NAME[p] });
    }
  }

  // Last, and every time: the rows show which pendulum is selected as well as
  // what each one is made of.
  paintArch();
}

// The letter switches its pendulum on and off the pivot. Selecting one is done
// by clicking anywhere else in its row, and resets nothing; switching one on or
// off does reset the world, so that every chain shares one t = 0, and so it asks
// first.
//
// The click stops here. The row it sits in treats a click as either "select me"
// or, when the pendulum is off, "put me back" — and both of those would fire a
// second time behind this one, the off case raising a second dialog.
for (let p = 0; p < MAX_PENDULUMS; p++) {
  TABS[p].addEventListener('click', (e) => {
    e.stopPropagation();
    if (hanging(chains[p])) confirmReset(() => setActive(p, false));
    else activate(p);
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

// The colour pair in the box's corner, the same one control in two buttons.
// Every point carries the g it was recorded under, so this only changes how what
// is already there is drawn — nothing is cleared and nothing is re-recorded.
const trailColours = { chain: el('colour-chain'), gravity: el('colour-gravity') };
for (const [mode, button] of Object.entries(trailColours)) {
  button.addEventListener('click', () => {
    trailColour = mode;
    for (const [key, b] of Object.entries(trailColours)) b.classList.toggle('on', key === mode);
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

// Writes a template string into a node with its {placeholders} replaced by
// spans, so a sentence whose clauses reorder between languages can still pick
// out the values inside it. Each value is [id, text], the id optional; the text
// goes in as a text node, never as markup.
function writeParts(node, key, vals) {
  node.textContent = '';
  for (const part of t(key).split(/(\{\w+\})/)) {
    if (!part) continue;
    if (!vals[part]) {
      node.appendChild(document.createTextNode(part));
      continue;
    }
    const [id, text] = vals[part];
    const span = document.createElement('span');
    if (id) span.id = id;
    span.textContent = text;
    node.appendChild(span);
  }
}

// Where the run starts and which phase the release lands in.
//
// Takes the value rather than reading `release`, because while the slider is
// dragged the note previews a release that has not been committed — the run is
// still flying the old one. See setRelease below.
function updateReleaseNote(v = release) {
  writeParts(el('release-note'), 'release.note', {
    '{start}': ['r-start', `t = ${Math.max(0, v - LEAD_IN)} s`],
    '{when}': ['r-when', `t = ${v} s`],
    '{where}': ['r-where', phaseInline(flightAt(v).ph)]
  });
}

// Repaints every translated string wholesale, so no label can be left behind in
// the old language.
function applyLang() {
  document.documentElement.lang = lang;
  document.title = t('page.title');

  for (const node of document.querySelectorAll('[data-i18n]')) node.textContent = t(node.dataset.i18n);
  for (const node of document.querySelectorAll('[data-i18n-title]')) node.title = t(node.dataset.i18nTitle);
  // For a control whose name is not written on the panel — the trail style
  // pair, which took the full width its label used to lead.
  for (const node of document.querySelectorAll('[data-i18n-aria]')) {
    node.setAttribute('aria-label', t(node.dataset.i18nAria));
  }

  // The switch shows the language it would take you to, not the one you are in.
  const other = lang === 'fr' ? 'en' : 'fr';
  const btn = el('lang');
  btn.textContent = other.toUpperCase();
  btn.title = LANG_TIP[other];
  btn.setAttribute('aria-label', LANG_TIP[other]);

  // The theme switch says its whole name in its tooltip, so it is translated
  // here as well as rewritten on a click.
  applyTheme();

  // The rest is written by JS, so it has to be asked to redraw. The State legend
  // and the switch tooltips carry the pendulum's letter, so they come from the
  // selection rather than from a bare key.
  el('play').textContent = t(running ? 'btn.pause' : 'btn.start');
  // The units the nine boxes below it are in, with the units themselves picked
  // out of the sentence.
  writeParts(el('arch-units'), 'arch.units', { '{L}': [null, 'm'], '{m}': [null, 'kg'] });
  // Set from JS rather than data-i18n-title, because it is on the row only while
  // the profile is flying — so it has to be rewritten here as well as there, or
  // a language switch mid-flight would leave it in the old one.
  el('row-g').title = flightOn ? t('flight.driven') : '';
  paintSelection();
  updateReleaseNote();
  updateFlightReadout();
}

// The switch carries no text of its own — a dot the colour of the page's text,
// which is light on dark and dark on light without a rule for either — so all
// there is to write is its name, and that names the theme a click gets you.
function applyTheme() {
  const tip = t(theme === 'dark' ? 'theme.light' : 'theme.dark');
  const btn = el('theme');
  btn.title = tip;
  btn.setAttribute('aria-label', tip);
}

function setFlight(on) {
  flightOn = on;
  flightTime = 0;
  el('flight-on').checked = on;
  el('flight').classList.toggle('on', on);
  // Why the row above is greyed, on the row rather than on the input: a disabled
  // input does not reliably raise the pointer events a tooltip needs, and this
  // is there precisely while g is disabled.
  el('row-g').title = on ? t('flight.driven') : '';
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
// one t = 0. Chains that are not hanging are reset too, so a row shows the
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

// Dark by default, which is the palette :root carries; light is the one thing
// the attribute switches on. Everything the page draws in CSS follows from the
// custom properties alone, so the only work here is the handful of colours JS
// holds copies of.
el('theme').addEventListener('click', () => {
  theme = theme === 'dark' ? 'light' : 'dark';
  document.documentElement.dataset.theme = theme;
  readPalette();
  // The profile's static half is cached against its inputs, and a repaint of
  // the panel is what the colours it was painted in changed under it.
  layerKey = '';
  paintArch();
  applyTheme();
});

// No confirmation: the button says Reset. It pauses, because reset() does.
el('reset').addEventListener('click', reset);

// The announcement is here rather than inside clearTrails, which reset() already
// calls on its own way to announcing itself.
el('clear').addEventListener('click', () => { clearTrails(); worldChanged(); });

// Snapshot the live state into the initial-condition boxes, so an interesting
// configuration found mid-flight can be replayed. Every pendulum on the pivot,
// not the selected one alone: they were caught in that configuration together,
// and the box above shows all of them.
el('use-current').addEventListener('click', () => {
  for (const c of world) {
    for (let i = 0; i < c.n; i++) {
      c.th0[i] = Number(wrapDeg(c.s[i]).toFixed(1));
      c.om0[i] = Number(deg(c.s[c.n + i]).toFixed(1));
      INIT[c.slot][i].th.value = c.th0[i];
      INIT[c.slot][i].om.value = c.om0[i];
    }
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

// Every pendulum takes its rods and its initial conditions from its own boxes
// rather than from DEFAULTS, so the markup and the load cannot disagree. All
// three are on the panel at once now, so all three are stated there; only the
// link counts still come from the table, since a link count is said by the markup
// only in which boxes it hides.
for (let p = 0; p < MAX_PENDULUMS; p++) {
  for (let i = 0; i < MAX_LINKS; i++) {
    chains[p].L[i] = num('L-' + (p + 1) + '-' + (i + 1), DEFAULTS[p].L[i]);
    chains[p].m[i] = num('m-' + (p + 1) + '-' + (i + 1), DEFAULTS[p].m[i]);
    chains[p].th0[i] = numOf(INIT[p][i].th, i === 0 ? 90 : 0);
    chains[p].om0[i] = numOf(INIT[p][i].om, 0);
  }
}

// Friction is the one parameter still read from a single box, because it is the
// one that follows the selection, and the panel opens on A.
chains[0].b = num('b', 0.001);

// B and C are fully built at load rather than at switch-on, so one that is
// switched off and then tuned is not overwritten when it comes back. Only their
// trails are left, since a checkbox cannot be derived from a link count in the
// markup.
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
