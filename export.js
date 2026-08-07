'use strict';

// --- Trace export ----------------------------------------------------------
//
// Saves the pattern traced during one section of the flight cycle — the whole
// apesanteur segment, say — as a PNG or an SVG holding nothing but the traces.
//
// Loaded after index.js. Both are classic scripts, so index.js's top-level
// bindings — world, PHASES, cycle, startTime, trailSeconds, simTime, flightTime,
// COLOUR, trailStyle, trailColour, shadeOf, traced, firstTraced, the tape
// functions, el, t and fmt — are readable here without either file exporting
// anything.
//
// Nothing here runs per frame; every function hangs off a click or a change.
//
// A span is two times rather than one of the nine named phases, since the useful
// pictures cross them — apesanteur with the injection that threw the piece into
// it, or the middle of a long baseline. The panel names whichever phases the
// span covers and index.js draws it on the profile. Points come from the export
// tape, not the drawn trail, so a span stays available from the moment it has
// been flown; see index.js's tape section.

const EXPORT_R = 2048;        // output square, in pixels (PNG) or user units (SVG)
const EXPORT_PAD = 0.02;      // margin, as a fraction of the square
const EXPORT_STROKE = 0.0015; // trace width, ditto — about 3 units at 2048
const EXPORT_DOT = 0.0016;    // dot radius, ditto

const fromBox = el('export-from');
const toBox = el('export-to');
const phaseNote = el('export-phases');
const segStatus = el('export-status');
const autoNote = el('export-auto');
const pngButton = el('export-png');
const svgButton = el('export-svg');
const autoBox = { png: el('auto-png'), svg: el('auto-svg') };

// --- The span --------------------------------------------------------------

// The two boxes, in cycle seconds, clamped to a cycle the shortcut can shorten
// under them. Read on every use: the boxes are the state.
function spanOf() {
  const C = cycle();
  const read = (box, dflt) => {
    const v = parseFloat(box.value);
    return Number.isFinite(v) ? clamp(v, 0, C) : dflt;
  };
  return [read(fromBox, 0), read(toBox, C)];
}

// The phases the span touches, in order, adjacent repeats collapsed: two of the
// nine are hypergravity and two baseline, and a span crossing from one to the
// next would otherwise name it twice.
function spanPhases() {
  const C = cycle();
  const [t0, t1] = spanOf();
  const out = [];
  for (const ph of PHASES) {
    if (Math.min(ph.t1, C) <= t0 || ph.t0 >= t1) continue;
    if (out[out.length - 1]?.key !== ph.key) out.push(ph);
  }
  return out;
}

// --- Finding the points ----------------------------------------------------

// The most recent completed pass over the span, as a window in flight time, or
// null when the cycle has not reached the end of the span or the span is empty.
// live marks the window that comes from the drawn trail rather than the tape:
// with the profile off there are no cycle times, so what there is to save is
// whatever is on the screen.
function segmentWindow() {
  if (!flightOn) {
    return { from: -Infinity, to: Infinity, cycleNo: 0, span: trailSeconds, live: true };
  }
  const C = cycle();
  const [t0, t1] = spanOf();
  if (t1 <= t0) return null;
  const k = Math.floor((flightTime - t1) / C);
  if (k < 0) return null;
  return { from: k * C + t0, to: k * C + t1, cycleNo: k + 1, span: t1 - t0, live: false };
}

// Every traced bob's points inside the window, as world-metre [x, y, t, g].
// Reads and mutates nothing.
//
// Nothing is thinned: every point recorded is a point exported, in both styles.
// The budget is fixed — 60 Hz, a span of at most one 180 s cycle, at most six
// traced bobs — so the ceiling is 64 800 points and a 1.05 MB SVG.
//
// The tape stamps points with the chain's own clock while the window is in
// flight time, and the two differ by exactly startTime(): index.js keeps
// flightTime − startTime() === simTime, so a point stamped c.t was recorded at
// startTime() + c.t. Chains sit up to their own sub-step apart (232 µs worst
// measured), smaller than the spacing between two points.
function collect(w) {
  const rows = [];
  for (const c of world) {
    for (let i = firstTraced(c); i < c.n; i++) {
      if (!traced(c, i)) continue;
      const pts = w.live
        ? c.trails[i]
        : tapePoints(c.tape[i], w.from - startTime(), w.to - startTime());
      if (pts.length) {
        // The chain's own colour, which is the whole answer in the chain mode
        // and the fallback the gravity mode never reaches; runsOf() decides.
        rows.push({ chain: c, bob: i + 1, colour: COLOUR.chain[c.slot], pts });
      }
    }
  }
  return rows;
}

// What the panel is asking to be drawn, whether or not it has anything to draw
// yet. Distinguishes "no trace in this segment" from "nothing traced at all".
function anyTraced() {
  for (const c of world) {
    for (let i = firstTraced(c); i < c.n; i++) if (traced(c, i)) return true;
  }
  return false;
}

// How far back the recording reaches, in chain time — the oldest point any
// traced bob still holds, and Infinity when none of them holds anything. The
// tape is rewound by a reset and by every parameter change, so this is what
// says whether a section that has been flown was also recorded.
function recordedFrom() {
  let oldest = Infinity;
  for (const c of world) {
    for (let i = firstTraced(c); i < c.n; i++) {
      if (traced(c, i)) oldest = Math.min(oldest, tapeStart(c.tape[i]));
    }
  }
  return oldest;
}

// --- The frame -------------------------------------------------------------

// The stage's mapping: pivot centred, scaled to the largest reach in the world
// rather than to what this segment covers, so two exports of the same world are
// at the same scale. No bob radius in the margin, unlike draw(): no bobs here.
function frameOf() {
  let reach = 0;
  for (const c of world) {
    let r = 0;
    for (let i = 0; i < c.n; i++) r += c.L[i];
    if (r > reach) reach = r;
  }
  const R = EXPORT_R;
  const scale = (R / 2 - R * EXPORT_PAD) / reach;
  return {
    R,
    scale, // pixels per metre; toX and toY are the only users left
    stroke: R * EXPORT_STROKE,
    dot: R * EXPORT_DOT,
    toX: (x) => R / 2 + x * scale,
    toY: (y) => R / 2 - y * scale
  };
}

// --- Colour ----------------------------------------------------------------

// The trail colour control reaches the file as well as the stage: a mode is a
// way of drawing points already recorded, and the two pictures are of the same
// points, so they cannot disagree about what colour those points are.
//
// A row cut into runs of one colour, which is all either format needs to know
// about the modes. In the chain mode that is the row itself, in one run. In the
// gravity mode it is one run per step of the ramp — the same cut drawTrail makes
// on the stage, minus the fade, which an export does not have: a run here lasts
// as long as the level is flat instead of stopping at a chunk boundary.
function runsOf(row) {
  if (trailColour !== 'gravity') return [{ colour: row.colour, pts: row.pts }];
  const dots = trailStyle === 'dots';
  const out = [];
  let a = 0;
  let shade = shadeOf(row.pts[0][3]);
  for (let i = 1; i < row.pts.length; i++) {
    const next = shadeOf(row.pts[i][3]);
    if (next === shade) continue;
    // The line hands the point at the step on to the next run as well, so the
    // segment across it is drawn once, in the older shade. Shared, not repeated:
    // dots take the point once or it is drawn twice over.
    out.push({ colour: shade, pts: row.pts.slice(a, dots ? i : i + 1) });
    a = i;
    shade = next;
  }
  out.push({ colour: shade, pts: row.pts.slice(a) });
  return out;
}

// --- SVG -------------------------------------------------------------------

const esc = (s) => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');

// The parameters that produced the pattern, for the SVG's <desc>. PNG has no
// equivalent, since a text chunk would mean writing a PNG encoder.
function describe(w, rows) {
  const lines = [];
  if (w.live) {
    lines.push(`Pendulum Simulator — current trail, ${trailSeconds} s to t = ${simTime.toFixed(1)} s`);
  } else {
    const [t0, t1] = spanOf();
    const names = spanPhases().map(phaseName).join(' + ') || '—';
    lines.push(`Pendulum Simulator — t = ${t0.toFixed(1)}–${t1.toFixed(1)} s `
      + `of cycle ${w.cycleNo}, ${names}`);
  }

  for (const c of world) {
    const mine = rows.filter((r) => r.chain === c);
    if (!mine.length) continue;
    const arr = (a, dp) => '[' + a.slice(0, c.n).map((v) => v.toFixed(dp)).join(', ') + ']';
    lines.push(`pendulum ${chainName(c)}, bob ${mine.map((r) => r.bob).join(' and ')}: `
      + `n=${c.n} L=${arr(c.L, 2)} m=${arr(c.m, 2)} b=${c.b} `
      + `th0=${arr(c.th0, 1)} om0=${arr(c.om0, 1)}`);
  }

  lines.push(flightOn ? 'g: driven by the flight profile'
    : `g: ${env.g.toFixed(2)} m/s^2`);
  lines.push(`style: ${trailStyle}, colour: ${trailColour}`);
  return lines.join('\n');
}

// Traces only — no rods, bobs, pivot or background rect, so the pattern drops
// onto whatever it is placed on. Solid rather than faded.
function toSVG(rows, meta) {
  const f = frameOf();
  const n = (v) => v.toFixed(2);
  const dots = trailStyle === 'dots';

  const body = rows.flatMap(runsOf).map((row) => {
    if (dots) {
      const r = n(f.dot);
      return `<g fill="${row.colour}">`
        + row.pts.map((p) => `<circle cx="${n(f.toX(p[0]))}" cy="${n(f.toY(p[1]))}" r="${r}"/>`).join('')
        + '</g>';
    }
    const d = row.pts
      .map((p, i) => (i ? 'L' : 'M') + ' ' + n(f.toX(p[0])) + ' ' + n(f.toY(p[1])))
      .join(' ');
    return `<path d="${d}" fill="none" stroke="${row.colour}" stroke-width="${n(f.stroke)}"`
      + ' stroke-linecap="round" stroke-linejoin="round"/>';
  }).join('\n  ');

  return '<?xml version="1.0" encoding="UTF-8"?>\n'
    + `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${f.R} ${f.R}"`
    + ` width="${f.R}" height="${f.R}">\n`
    + `  <desc>${esc(meta)}</desc>\n  ${body}\n</svg>\n`;
}

// --- PNG -------------------------------------------------------------------

// The same runs through the same mapping, so the two formats are one picture.
// Transparent: nothing is painted but the traces.
function toPNG(rows) {
  const f = frameOf();
  const cv = document.createElement('canvas');
  cv.width = f.R;
  cv.height = f.R;
  const c2 = cv.getContext('2d');
  const dots = trailStyle === 'dots';

  c2.lineWidth = f.stroke;
  c2.lineCap = 'round';
  c2.lineJoin = 'round';

  for (const row of rows.flatMap(runsOf)) {
    c2.fillStyle = row.colour;
    c2.strokeStyle = row.colour;
    c2.beginPath();
    for (let i = 0; i < row.pts.length; i++) {
      const x = f.toX(row.pts[i][0]);
      const y = f.toY(row.pts[i][1]);
      if (dots) {
        // A fresh subpath at each dot, or the arcs are joined by a line.
        c2.moveTo(x + f.dot, y);
        c2.arc(x, y, f.dot, 0, Math.PI * 2);
      } else if (i === 0) {
        c2.moveTo(x, y);
      } else {
        c2.lineTo(x, y);
      }
    }
    if (dots) c2.fill(); else c2.stroke();
  }

  return new Promise((resolve) => cv.toBlob(resolve, 'image/png'));
}

// --- Saving ----------------------------------------------------------------

function download(blob, name) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  a.click();
  URL.revokeObjectURL(a.href);
}

// Local wall-clock time as YYYYMMDD-HHMM. Local rather than UTC, since the name
// is read against the session it was made in. Minutes, not seconds: two exports
// in the same minute are told apart by the rest of the name.
function stamp(d = new Date()) {
  const p = (v) => String(v).padStart(2, '0');
  return `${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}`
    + `-${p(d.getHours())}${p(d.getMinutes())}`;
}

// When it was saved, then what it is: the span within the cycle and which pass
// over it. The stamp leads, so a directory sorts into the order the work
// happened in. Bounds are cycle-relative, not absolute flight times, and the
// phase key appears only when the span sits inside a single phase.
function fileName(w, ext) {
  if (w.live) return `${stamp()}-pendulum-trail-${simTime.toFixed(0)}s.${ext}`;
  const [t0, t1] = spanOf();
  const phs = spanPhases();
  const key = phs.length === 1 ? phs[0].key + '-' : '';
  return `${stamp()}-pendulum-${key}${t0}-${t1}s-cycle${w.cycleNo}.${ext}`;
}

// Returns whether anything was written, which is what the automatic path needs
// to know before it marks a pass as done.
async function save(ext) {
  const w = segmentWindow();
  if (!w) return false;
  const rows = collect(w);
  if (!rows.length) return false;

  const name = fileName(w, ext);
  if (ext === 'svg') {
    const svg = toSVG(rows, describe(w, rows));
    download(new Blob([svg], { type: 'image/svg+xml' }), name);
  } else {
    download(await toPNG(rows), name);
  }
  return true;
}

// --- Automatically -------------------------------------------------------

// The pass already saved, so one completed span produces one file however often
// the check below runs. Set to the pass in progress when a format is armed, so
// arming means "from the next one".
let autoDone = 0;

const autoWanted = () => ['png', 'svg'].filter((k) => autoBox[k].checked);

// Polled rather than called on the frame the span ends: a span is defined by the
// seconds it covers, so noticing a quarter of a second late writes the same
// file, and this keeps the frame loop free of anything of ours.
const AUTO_POLL = 250;

async function autoTick() {
  const want = autoWanted();
  if (!want.length || !flightOn) return;
  const w = segmentWindow();
  if (!w || w.live || w.cycleNo <= autoDone) return;
  // Marked done before the first await: a PNG encode is asynchronous, and two
  // polls either side of it would write the pass twice. Marked even when nothing
  // can be written, so a span the tape does not hold is one skipped pass rather
  // than a collect() four times a second for the rest of the cycle.
  autoDone = w.cycleNo;
  if (anyTraced() && w.from - startTime() >= recordedFrom()) {
    for (const ext of want) await save(ext);
  }
  paintStatus();
}

setInterval(autoTick, AUTO_POLL);

// --- The panel -------------------------------------------------------------

// Exactly one state, recomputed on the events that can change it rather than per
// frame. Nothing it shows counts down, so it does not go stale between them.
function paintStatus() {
  const C = cycle();
  // With no profile there are no cycle times, so the two boxes have nothing to
  // mean; what there is to save is then whatever the trail is holding.
  for (const box of [fromBox, toBox, autoBox.png, autoBox.svg]) box.disabled = !flightOn;
  fromBox.max = C;
  toBox.max = C;

  const [t0, t1] = spanOf();
  const good = flightOn && t1 > t0;

  // The mark on the profile is the same span as the boxes by construction,
  // because this is the only line that writes it.
  markSpan = good ? [t0, t1] : null;

  if (!flightOn) phaseNote.textContent = t('export.current');
  else if (!good) phaseNote.textContent = t('export.badspan');
  else phaseNote.textContent = `${t0}–${t1} s · ` + spanPhases().map(phaseName).join(' → ');

  let text;
  let ready = false;

  // The span's own length, independent of whether it has been flown, so one too
  // long to record says so now rather than after a cycle of waiting. Unreachable
  // while the tape holds two cycles and the boxes cannot express more than one;
  // the guard stays because TAPE_SECONDS could be lowered.
  const span = flightOn ? t1 - t0 : 0;

  const traces = anyTraced();
  const w = traces && span <= TAPE_SECONDS ? segmentWindow() : null;
  if (!traces) {
    text = t('export.none');
  } else if (span > TAPE_SECONDS) {
    text = fmt('export.short', { n: Math.ceil(span) });
  } else if (!w) {
    text = t('export.unflown');
  } else if (!w.live && w.from - startTime() < recordedFrom()) {
    // Flown but not recorded: a reset or a parameter change rewound the tape
    // after it. The cycle brings it round again, so say when.
    text = fmt('export.expired', { s: (w.to + cycle()).toFixed(0) });
  } else {
    const rows = collect(w);
    if (!rows.length) {
      // Completed but empty: the piece was clamped through the whole of it.
      text = t('export.empty');
    } else {
      // Points rather than pendulums, which the toggles above already say.
      // Nothing is dropped after this, so it is what the file will hold —
      // counted per row rather than per colour run, since the gravity mode's
      // runs share the point they meet at rather than each holding one.
      const pts = rows.reduce((a, r) => a + r.pts.length, 0);
      text = w.live
        ? fmt('export.readyNow', { n: pts })
        : fmt('export.ready', { n: pts, k: w.cycleNo });
      ready = true;
    }
  }

  segStatus.textContent = text;
  pngButton.disabled = !ready;
  svgButton.disabled = !ready;

  // When the next automatic save falls due, in flight time: the end of the span
  // in the pass after the one already written. A fixed number, not a countdown.
  autoNote.textContent = good && autoWanted().length
    ? fmt('export.autoNext', { s: (autoDone * cycle() + t1).toFixed(0) })
    : '';
}

// Everything the span can change, in one place: the mark on the profile, the
// phases named under the boxes, and what the next automatic save waits for. A
// new span re-arms, since the pass it has already been through is not one this
// asked to save.
function spanChanged() {
  const w = segmentWindow();
  autoDone = w && !w.live ? w.cycleNo : 0;
  paintStatus();
}

for (const box of [fromBox, toBox]) {
  box.addEventListener('input', spanChanged);
  box.addEventListener('change', spanChanged);
}

for (const k of ['png', 'svg']) autoBox[k].addEventListener('change', spanChanged);

pngButton.addEventListener('click', () => save('png').then(paintStatus));
svgButton.addEventListener('click', () => save('svg').then(paintStatus));

// Refreshed on the way in rather than on a timer: everything that can change the
// answer is a user action, and by the time the pointer is over the panel it has
// already happened.
el('export').addEventListener('pointerenter', paintStatus);

// The phases under the boxes are named in the current language. Registered after
// index.js's own listener, so the language has switched by the time this runs.
el('lang').addEventListener('click', paintStatus);

// Everything else arrives through index.js's announcement rather than a listener
// on the control that caused it, because a click on the flight switch or the
// shortcut may be waiting on a confirmation — a second listener would repaint
// from a world that has not changed yet, and not be called when the answer came.
//
// A re-run is also the moment the automatic export has nothing left to wait for.
worldHooks.push(() => {
  autoDone = 0;
  paintStatus();
});

paintStatus();
