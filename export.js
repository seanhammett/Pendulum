'use strict';

// --- Trace export ----------------------------------------------------------
//
// Saves the pattern traced during one section of the flight cycle — the whole
// apesanteur segment, say — as a PNG or an SVG holding nothing but the traces.
//
// Loaded after index.js. Top-level const and let in a classic script live in
// the shared global lexical environment, so world, PHASES, cycle, startTime,
// trailSeconds, simTime, flightTime, COLOUR, trailStyle, traced, firstTraced,
// the tape functions, el, t and fmt are all readable from here without index.js
// exporting anything.
//
// Nothing here runs per frame. Every function below hangs off a click or a
// change, so the frame budget is untouched.
//
// Spans are read from the export tape rather than from the drawn trail. The
// drawn trail is redrawn in full every frame, so it can only hold about a
// minute, and a 22 s section sat inside a 25 s trail was downloadable for the
// three seconds between finishing and having its start pruned. The tape holds
// two cycles and is not drawn, so a span stays available from the moment it has
// been flown. See index.js's tape section for the sizing.
//
// What is exported is two times rather than one of the nine named phases. The
// phases are the aircraft's divisions of the cycle, and they are often not the
// picture: the apesanteur with the injection that threw the piece into it, or
// the middle of a long baseline, are spans the list could not name. The phases
// are still what the seconds mean, so the panel names whichever of them the
// span covers, and index.js draws the span on the profile.

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

// The two boxes, in cycle seconds, clamped to a cycle that the shortcut can
// shorten under them. Read on every use rather than cached: the boxes are the
// state, so there is nothing to keep in step with them.
function spanOf() {
  const C = cycle();
  const read = (box, dflt) => {
    const v = parseFloat(box.value);
    return Number.isFinite(v) ? clamp(v, 0, C) : dflt;
  };
  return [read(fromBox, 0), read(toBox, C)];
}

// The phases the span touches, in order, with the repeats collapsed — two of
// the nine are called hypergravity and two baseline, and a span crossing from
// one hypergravity into the next would otherwise name it twice.
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

// The most recent completed pass over the span, as a window in flight time.
//
// Returns null when the cycle has not reached the end of the span yet, or when
// the span is empty. live marks the one window that comes from the drawn trail
// rather than the tape: with the profile off there are no cycle times at all,
// so what there is to save is whatever is on the screen.
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

// The points a drawn line would not miss. Walking forward, a point is dropped
// when it lies within tol of the straight line between the last point kept and
// the one after it. At a tolerance of a quarter of a stroke width the result is
// the same picture, and it costs a smooth arc most of its points while leaving
// a chaotic one nearly untouched — which is the right way round, since the
// points a chaotic trace holds are the ones carrying its shape.
//
// Cusps survive by construction: a turning point is the furthest thing there is
// from the chord drawn across it.
function thin(pts, tol) {
  if (pts.length < 3) return pts;
  const out = [pts[0]];
  let a = pts[0];
  for (let i = 1; i < pts.length - 1; i++) {
    const b = pts[i];
    const c = pts[i + 1];
    const dx = c[0] - a[0];
    const dy = c[1] - a[1];
    const chord = dx * dx + dy * dy;
    // Cross product over chord length is the perpendicular distance. A zero
    // chord means the kept point and the next one coincide, and then how far b
    // has strayed from the pair of them is the whole answer.
    const cross = (b[0] - a[0]) * dy - (b[1] - a[1]) * dx;
    const d2 = chord > 0
      ? (cross * cross) / chord
      : (b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]);
    if (d2 > tol * tol) {
      out.push(b);
      a = b;
    }
  }
  out.push(pts[pts.length - 1]);
  return out;
}

// Every traced bob's points inside the window, in world metres. Reads and
// mutates nothing. firstTraced is index.js's rule for which bobs a chain offers
// at all — the second one up, except on a single, where it is the only one.
//
// The tape stamps points with the chain's own clock and the window is in flight
// time, so the two differ by exactly startTime(). index.js keeps flightTime and
// simTime in lockstep — reset() sets flightTime to startTime() with every c.t at
// zero, and from then on the two advance by the same amount through the clamped
// phase and through free flight alike — so flightTime − startTime() === simTime,
// and a point stamped c.t was recorded at startTime() + c.t. Chains sit up to
// their own sub-step apart (worst measured 232 µs, 4 % of a frame), which is
// smaller than the spacing between two points and not worth correcting.
function collect(w) {
  const f = frameOf();
  // Dots are never thinned: there the spacing between marks is the speed of the
  // bob — crowding at each turning point, stretching through the bottom of the
  // swing — and dropping the close ones would erase exactly what that style is
  // drawn for.
  const tol = trailStyle === 'dots' ? 0 : f.stroke / 4 / f.scale;

  const rows = [];
  for (const c of world) {
    for (let i = firstTraced(c); i < c.n; i++) {
      if (!traced(c, i)) continue;
      const pts = w.live
        ? c.trails[i]
        : tapePoints(c.tape[i], w.from - startTime(), w.to - startTime());
      if (pts.length) {
        rows.push({
          chain: c,
          bob: i + 1,
          colour: COLOUR.chain[c.slot],
          pts: tol > 0 ? thin(pts, tol) : pts
        });
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

// The stage's own mapping: pivot centred, scaled to the largest reach in the
// world rather than to what this particular segment happens to cover. Two
// exports of the same world are then at the same scale whatever is in them,
// which is the comparison a shared pivot exists for. Cropping each pattern to
// its own bounding box would make a better single image and a worse set.
//
// No bob radius in the margin, unlike draw(): there are no bobs here.
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
    scale, // pixels per metre, so a tolerance in pixels can be stated in metres
    stroke: R * EXPORT_STROKE,
    dot: R * EXPORT_DOT,
    toX: (x) => R / 2 + x * scale,
    toY: (y) => R / 2 - y * scale
  };
}

// --- SVG -------------------------------------------------------------------

const esc = (s) => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');

// What the export is of. An exported pattern that does not say which parameters
// produced it is a picture rather than a result, and one string is the whole
// cost of it. There is no PNG equivalent — a text chunk means writing a PNG
// encoder — which is the asymmetry that makes SVG the format to keep.
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
  lines.push(`style: ${trailStyle}`);
  return lines.join('\n');
}

// Traces only — no rods, no bobs, no pivot, and no background rect, so the
// pattern drops onto whatever it is placed on. Solid rather than faded: the
// live view's fade from transparent to solid says "recent", which a still image
// has no use for.
function toSVG(rows, meta) {
  const f = frameOf();
  const n = (v) => v.toFixed(2);
  const dots = trailStyle === 'dots';

  const body = rows.map((row) => {
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

// The same rows through the same mapping, so the two formats are the same
// picture. Transparent: nothing is painted but the traces.
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

  for (const row of rows) {
    c2.fillStyle = row.colour;
    c2.strokeStyle = row.colour;
    c2.beginPath();
    for (let i = 0; i < row.pts.length; i++) {
      const x = f.toX(row.pts[i][0]);
      const y = f.toY(row.pts[i][1]);
      if (dots) {
        // A fresh subpath at each dot, or the arcs are joined by the very line
        // this style exists to do without.
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

// Local wall-clock time as YYYYMMDD-HHMM. Local rather than UTC because the
// name is read against the session it was made in — "which of these did we save
// after lunch" — and a directory of them from one flight is all in one zone
// anyway. Minutes, not seconds: two exports in the same minute are told apart
// by everything after the stamp.
function stamp(d = new Date()) {
  const p = (v) => String(v).padStart(2, '0');
  return `${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}`
    + `-${p(d.getHours())}${p(d.getMinutes())}`;
}

// When it was saved, then what it is: the span within the cycle and which pass
// over it. The stamp leads because it is what sorts a directory into the order
// the work happened in, which is the order an unattended run produces them in
// and the one nothing else in the name can give.
//
// Cycle-relative bounds rather than absolute flight times, since 35–57 names the
// span and 215–237 names an accident of when it was flown. The phase key is in
// there only when the span sits inside a single phase, where it is the name of
// the thing; across two of them it would be half a description.
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
// the check below runs. Set to the pass in progress when a format is switched
// on, because arming it means "from the next one" rather than "and the one that
// went past while I was reading the panel".
let autoDone = 0;

const autoWanted = () => ['png', 'svg'].filter((k) => autoBox[k].checked);

// Watching the flight clock rather than being called on the moment it crosses
// the end of the span. index.js runs nothing of ours per frame and this does not
// change that: a span is defined by the seconds it covers, so noticing that it
// has finished a quarter of a second late writes exactly the same file. Four
// times a second is also slow enough that the collect() below is not a cost
// worth measuring, and it only ever runs at all once a pass has completed.
const AUTO_POLL = 250;

async function autoTick() {
  const want = autoWanted();
  if (!want.length || !flightOn) return;
  const w = segmentWindow();
  if (!w || w.live || w.cycleNo <= autoDone) return;
  // Marked done before the first await, not after both saves: a PNG encode is
  // asynchronous, and two polls landing either side of it would write the pass
  // twice. Marked even when nothing can be written, so a span the tape does not
  // hold is one skipped pass rather than a collect() four times a second for
  // the rest of the cycle.
  autoDone = w.cycleNo;
  if (anyTraced() && w.from - startTime() >= recordedFrom()) {
    for (const ext of want) await save(ext);
  }
  paintStatus();
}

setInterval(autoTick, AUTO_POLL);

// --- The panel -------------------------------------------------------------

// Exactly one state, recomputed on the events that can change it rather than
// per frame — which is what costs the frame loop nothing. Nothing it shows
// counts down, so between those moments it does not go stale: a span that is
// ready stays ready, and the time the next automatic save falls due is a fixed
// number rather than a countdown to one.
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

  // Length of the span itself, independent of whether it has been flown, so
  // that a span too long to record says so now rather than after a cycle of
  // waiting to find out. Two cycles of tape against one cycle of profile means
  // only a span longer than the cycle can trip this, and the boxes cannot
  // express one — the guard stays because the tape length is a constant that
  // could be lowered.
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
    // Flown, but not recorded: the tape was rewound after it by a reset or a
    // parameter change. The cycle brings it round again, so say when rather
    // than just refusing.
    text = fmt('export.expired', { s: (w.to + cycle()).toFixed(0) });
  } else {
    const rows = collect(w);
    if (!rows.length) {
      // Completed but empty: the piece was clamped through the whole of it.
      text = t('export.empty');
    } else {
      // Points rather than pendulums: which chains are in it is already said
      // by the toggles right above, and a count would have to inflect. The
      // count is the thinned one, so it is what the file will hold.
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
  // in the pass after the one already written. It is a fixed number rather than
  // a countdown, so it stays true without being redrawn.
  autoNote.textContent = good && autoWanted().length
    ? fmt('export.autoNext', { s: (autoDone * cycle() + t1).toFixed(0) })
    : '';
}

// Everything the span can change, in the one place: the mark on the profile,
// the phases named under the boxes, and what the next automatic save is waiting
// for. Arming from here rather than only from the toggles is deliberate — a new
// span is a new thing to wait for, and the pass it has already been through is
// not one this asked to save.
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

// Everything that can change the answer is a user action, so the status is
// refreshed on the way in rather than on a timer: by the time the pointer is
// over the panel, whatever was clicked elsewhere has already happened.
el('export').addEventListener('pointerenter', paintStatus);

// The phases under the boxes are named in the current language, and nothing
// else on the page will repaint them. Registered after index.js's own listener,
// so the language has already switched by the time this runs.
el('lang').addEventListener('click', paintStatus);

// Everything else arrives through index.js's announcement rather than through a
// listener on the control that caused it. It has to: a click on the flight
// switch or the shortcut may now be waiting on a confirmation, so a second
// listener on the same control would repaint the panel from a world that has
// not changed yet — and would not be called at all when the answer finally came.
//
// The world starting again is also the moment the automatic export has nothing
// left to wait for, so the count of passes already written goes back to none.
worldHooks.push(() => {
  autoDone = 0;
  paintStatus();
});

paintStatus();
