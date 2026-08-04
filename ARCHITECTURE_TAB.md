# Architecture box — rework

Plan for replacing the Architecture fieldset's controls with the three-row
graphical form sketched in `ref/drawing.svg`. The physics, the canvas and the
reset semantics are untouched. Two other boxes are drawn into it: the Flight
simulator gives up its `g` row, and Trails picks up changes of its own, noted
at the foot.

`ref/arch-box.svg` draws the result in place: the whole control column at the
panel's real scale, with every other box where headless Chrome measures it on
the live page, so the new one is shown against the column it has to live in. It
holds the load defaults, with A selected and C switched off. It has since been
marked up by hand, and this plan follows those marks — read to the column grid
rather than to the pixel.

## What changes

| | Now | After |
|---|---|---|
| A / B / C strip | selects which pendulum the box edits; lit = selected, dot = hanging | activates/deactivates that pendulum; lit = **active** (on the pivot) |
| `Off\|Single\|Double\|Triple` | one control for hanging + link count | **gone** — each row carries its own `add L…` / `remove L…` button |
| `L1…L3`, `m1…m3` slider rows | one set, following the selected tab | **three row groups, one per pendulum, all visible at once** |
| Total length | not a control | new `LT` box per pendulum, coupled to the links |
| `g` (gravity) | a row in the Flight simulator box | **moves here**, straight under the tabs |
| `b` (friction) | a row at the foot of this box | **moves up** under `g`; still follows the selection |
| Units | `m` / `kg` on each slider row | one muted line under `b`, covering the grid below |
| Selection | which tab is lit | a lit surround around the pendulum's own row |
| An off pendulum | settings still editable through its tab | greyed figure, boxes hidden, the row itself switches it back on |

## Layout

Fieldset inner width is ~292 px (320 px panel − padding/border). The box reads
top to bottom as *which pendulums · what acts on them · what they are made of*:

```
 [    A    ] [    B    ] [    C    ]     lit = on the pivot

 g m/s²   ━━━━━━●━━━━━━━━━━   [9.81 ]   ← greyed; hover says why
 b N·m·s  ━●━━━━━━━━━━━━━━━   [0.001]
 Lengths in m, masses in kg              ← the grid's units

 ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━●   A · 1 link
 [add L2]                            L1 [0.994]   ← L1 is the total
                                     m1 [2.00 ]

 ●━━━━━━━━━━━━━━━━━━━━━━●━━━━━━━━━━━━━━━━━━━━━●   B · 2 links
 [add L3]    L1 [0.500]  L2 [0.494]  LT [0.994]
             m1 [4.00 ]  m2 [2.00 ] [remove L2]

 ●━━━━━━━━━━━━━━━━━━━●━━━━━━━━━━━━━━●━━━━━━━━━●   C · 3 links
 L1 [0.445]  L2 [0.331]  L3 [0.218]  LT [0.994]
 m1 [4.00 ]  m2 [1.80 ]  m3 [1.00 ] [remove L3]
```

Each pendulum gets one `.arch-row`: a figure spanning the full width, then a
4-column grid of number boxes under it, label to the left of each box.

Column rules, exactly as the sketch places them:

- 4 equal columns. The L row is **right-aligned**, `LT` always in column 4.
  `n = 1` has no separate total — its single `L1` box *is* the `LT` box and
  takes column 4 with the total's accent border.
- Each `m_i` sits directly under its `L_i`.
- **add** takes column 1 of the L row, free whenever `n < 3`.
- **remove** takes column 4 of the m row, free whenever `n > 1`.
  So the two step buttons need no space of their own and never collide.
- Each step button fills its **whole cell** — the label slot as well as the box
  slot, 69 px — and **names the link it moves**: `add L{n+1}`, `remove L{n}`.
  So a single reads `add L2`, a double `add L3` and `remove L2`, a triple
  `remove L3` alone. Muted 11 px on a `--bg` fill with a `--line` border, the
  same quiet button the trail toggles and the auto-export pair already use.
  Add is flush with the grid's left edge, remove with its right.
- `LT` (and `L1` on a single) carries the accent border, as in the sketch.
- No sliders in the pendulum rows — boxes only. Lengths 3 dp, masses 2 dp.
- The 4 columns leave no room for the `m` / `kg` units the current rows carry,
  so they go in one muted line above the rows, in the `.release-note` idiom:
  units in `--text`, the rest muted. One line covers all nine boxes, which
  eighteen tooltips would not do as well.

## The figure

Inline `<svg>`, no `viewBox` (user units = CSS px), redrawn on edit and on
resize. Straight and horizontal — this is architecture, not live state, so it
does not follow the angles. It mirrors `draw()` in `index.js` at panel scale:

| | canvas | panel figure |
|---|---|---|
| rod | white `--rod`, `max(2, size·0.006)` | white, 3 px |
| core | chain colour, `rodW · CORE_RATIO` | chain colour, `max(1, 3·0.34)` |
| bob | chain colour, `bobR · clamp(∛m, 0.6, 1.7)` | same law, `bobR = 6` → 3.6–10.2 px |
| bob ring | `--rod`, 2 px, stroked over the fill | `--rod`, 1.5 px, same |
| pivot | `--text`, `bobR · 0.45` | `--text`, 2.7 px |

Row height 26 px, so the heaviest bob (10.2 px radius + ring) clears it.

Scale is **shared across the three rows** — px/m from the largest total in the
box, hanging or not — exactly as the canvas scales every chain to the largest
reach. A shorter pendulum therefore draws shorter, and an `LT` edit is visible.
At the defaults all three totals are 0.994 m, so all three fill the width.

## Selected, active, off

Three states, and the row says which one it is in:

| | figure | boxes | surround |
|---|---|---|---|
| selected | full | shown | 1 px `--accent`, 7 % accent fill, 5 px radius |
| active, not selected | full | shown | none |
| off | grey | **hidden** | none at rest; accent border, no fill, on hover |

The surround is a lit tab drawn around the whole row rather than around a
letter — the same border, fill and radius `.tab.on` carries — so the strip and
the rows say *selected* the same way. It sits at the fieldset's padding edge,
4 px outside the content, which gives the figure and the boxes room inside it.

An off pendulum greys: rod and bobs in `#565e73`, the grey `index.js` already
uses for the flight curve when nothing is flying it, bobs filled `--line` with a
ring in that grey, and the pivot with them. The chain's colour is off the row
entirely; the tab above still carries it as a hollow dot.

Its boxes go with it, and the empty row becomes the switch: click the grey
figure and the pendulum goes back on the pivot. Hovering answers with the accent
border every other clickable thing on the panel answers with, and no fill —
which is what keeps it apart from the selected row's lit fill.

The one thing this costs: an off pendulum's lengths and masses can no longer be
edited without switching it on first, which today's tabs allow, and switching on
goes through `confirmReset` like every other change to the pivot. Setting up a
pendulum before hanging it is therefore two steps rather than one. The
alternative is to keep the boxes and grey only the figure, which reads less
plainly as *off* and costs the column another 56 px per off pendulum.

The column grows about **77 px** net. The Architecture box itself roughly
doubles — three pendulums drawn at once where one used to be, with `g` and the
units line added above them — but ~63 px of that is `g` arriving from the
Flight simulator box rather than new, six slider rows and the links switch come
off, the driven note becomes a tooltip, and `Clear trails` stops being a row of
its own. An off pendulum costs only its 26 px figure, so switching two off
leaves the box shorter than it is today.

## Length coupling

`T = Σ L_i`. `LMIN = 0.1` per link, total capped at `LMAX_T = 3` — today's
per-link maximum, so the stage's extent is unchanged. A link's own maximum
becomes `T − LMIN·(n−1)`: a link can never exceed its chain.

**Total changed to `T′`** — every link scales together, ratios held:

```
T′ ← clamp(T′, LMIN·n, LMAX_T)
k  = T′ / T
L_i ← k · L_i           for all i
```

**Link `i` changed to `v`** — the others take up the slack, `T` unchanged:

```
v ← clamp(v, LMIN, T − LMIN·(n−1))
R = T − v                       budget left for the rest
S = Σ_{j≠i} L_j                 always > 0, every L_j ≥ LMIN
L_j ← L_j · (R / S)             for j ≠ i
L_i ← v
```

`n = 1` is the degenerate case: the one link *is* the total, so editing it moves
`T`, which is why the sketch puts it in the total's column.

Values stay full precision internally and are only rounded for display, so
nudging one box repeatedly does not walk the total. Every box in the row is
rewritten from the model after each edit, so the siblings visibly move.

## Link count

The two buttons preserve the total, matching the rest of the box:

```
add:     L_i ← L_i · n/(n+1)  for all i,  then append L_{n+1} = T/(n+1)
remove:  drop L_n,  then L_i ← L_i · T/(T − L_n)
```

The new link's mass comes from the chain's stored `m[n]` (the DEFAULTS/markup
value), as unused links already hold today. Both re-run from initial conditions
through `confirmReset`, exactly as the links switch does now, and both call
`tipOnly()` since the chain has a new tip.

Both buttons name the link they move: `add L{n+1}`, `remove L{n}`. The label
changes with the row, so a double offers `add L3` and `remove L2` and a triple
only `remove L3`. Naming the link ties each button to the box it will create or
take away, one column over.

## g and b

The two forces move to the top of the box, straight under the tab strip and
above the three chains they act on:

- **`g`** comes out of the Flight simulator box, the same `.row` and the same
  disabled-and-greyed state while a profile drives it.
- **`b`** comes up from the foot of this box to sit under `g`. It still follows
  the selection: it is the selected pendulum's friction, not a global.
- Then the **units line**, then the three pendulum rows.

**`g is driven by the flight profile` stops being a line in the column** and
becomes a tooltip on the `g` row, shown only while a profile is flying — the
same thing it says today, just not taking 28 px of column to say it. Two
things follow from that:

- The tooltip goes on the **row**, not on the input. A `disabled` input does
  not reliably raise the pointer events a `title` needs, and `g` is disabled
  precisely when the tooltip has something to say.
- `title` is set from JS rather than `data-i18n-title`, since it must appear
  and disappear with the flight state; `applyLang()` has to rewrite it too, or
  it will hold the old language after a switch.

The trade-off is that a greyed control now explains itself only on hover. It is
greyed at exactly the moment the Flight simulator box above it is lit and
running, so the cause is on screen even when the sentence is not.

`g` is global and `b` is per-pendulum, which the box does not say. It did not
say so before either — `b` sat at the foot with the same ambiguity — and the
selected row's lit surround is the thing that answers it, since `b` visibly
belongs to whichever row is lit. Worth a tooltip either way.

Moving `g` costs the Flight simulator box nothing and saves it ~63 px; the two
boxes stop being separated by the one control that belongs to both.

## Activate / deactivate

Clicking A, B or C splices its chain in or out of `world` through
`confirmReset` — the same path `setLinks(0)` takes today, including the guard
that the last chain on the pivot cannot be switched off (that button disables).
Clicking an off pendulum's grey figure is the same call, so the strip and the
row are two handles on one control. `.tab.on` now means active. The dot keeps
its current filled/hollow behaviour — redundant with the lit state, but it is
what the strip looks like today and the same mark still reads across the tab and
the Energy row.

## Trails box

Marked up in the same pass, and independent of everything above — it can land
on its own. Three rows instead of four, and one slider fewer:

```
 A [1]   B [2]   C [2][3]

 [       Dots       ] [       Lines      ]      ← full width, no label

 [ Clear ]  ━━●━━━━━━━━  Length s [25]
```

- **Style** loses its `Style` label; the `.seg` takes the full inner width, two
  halves of ~144 px. `Line` becomes **`Lines`**, which is what it draws.
- **Length** keeps its slider, shortened to ~98 px. The row goes from `.row`'s
  `58px | 1fr | 62px` to four cells: `Clear` in the first, the shortened
  slider, then `Length s`, then the number box it already has. The label moves
  from the head of the row to just left of its value, which is the one thing
  here that departs from every other `.row` on the panel — worth a look in the
  browser before it sticks.
- **`Clear trails`** stops being a full-width `button.wide` at the foot and
  becomes that compact `Clear` at the head of the Length row. Its label loses
  the noun, which the legend two rows up already supplies.
- The box comes out ~33 px shorter.

The trail toggle groups are nudged apart by a few px in the markup; that is
even spacing, not a new rule.

## Work

- **index.html** — replace `.seg.arch` and the six `L`/`m` slider rows with the
  three `.arch-row` groups: nine lengths, nine masses, three totals, and the six
  step buttons. All three pendulums' defaults now live in the markup, so
  `DEFAULTS` is read back for every slot at load, not just A. Move the `g` row
  out of the flight fieldset to just under the tabs and `b` up beneath it;
  delete `#g-driven` entirely; add the units note.
- **index.css** — `.arch-row`, `.arch-row.sel` (the lit surround, sharing
  `.tab.on`'s border and fill), `.arch-row.off`, `.arch-fig`, `.arch-grid`,
  `.arch-cell`, `.arch-cell.total`, `.arch-step`. `.driven` loses its only
  user and can go with it; the units note reuses `.release-note`.
- **index.js** — tab handlers toggle active instead of selecting;
  `setLinks(count)` → `setLinkCount(slot, n)`; new `setTotal`/`setLink` coupling;
  `linkButtons`, `setL`, `setM` (slider pairs) replaced by box binders per slot;
  `paintSelection()` gains `paintArch()` for the figures and boxes. In
  `setFlight` (≈ line 1438) `el('g-driven').hidden = !on` becomes a `title`
  written onto the `g` row; `applyLang()` must rewrite it. `g` keeps its own
  bindings — it only changes parent — so check nothing walks the flight
  fieldset's children to find it.
- **i18n.js** — drop `arch.off/single/double/triple`, `arch.links`,
  `arch.links.tip`; add the two step-button labels (both take the link number,
  so they interpolate), the units line, plus tooltips for total, length, mass
  and the active toggle, in both tables.
- **README.md** — the panel walkthrough (≈ lines 239, 310, 324, 338) describes
  the links switch, tab-as-selection, and `g` living in the flight box.

Trails, separable from the above:

- **index.html** — drop the `Style` label; move `#clear` into the length row as
  its first child, off `.wide`, and move the `Length` label after the range.
- **index.css** — a four-cell variant of `.row` for the length row; `.seg`
  already stretches once the label leaves the style row.
- **i18n.js** — `trails.line` → `Lines` / `Lignes`; `trails.clear` → `Clear` /
  `Effacer`; `trails.style` becomes tooltip-only.
- **index.js** — untouched: the range keeps its id and its pairing.

## Decided

1. **Step buttons** — remove takes the m row's free column-4 cell, add the L
   row's column-1 cell, each filling its whole cell and naming the link it
   moves: `add L{n+1}`, `remove L{n}`.
2. **Selection** — clicking anywhere in an active row, or focusing any box in
   it, selects that pendulum. The row lights: the same border and fill as a lit
   tab, around the whole row. The State legend keeps naming the letter.
   Selecting resets nothing, as today.
3. **Off** — greyed to `#565e73`, boxes hidden, and the row itself is what
   switches the pendulum back on.
4. **`g` and `b`** — both at the top of the box, `g` in from the flight
   simulator, `b` up from the foot, above the chains they act on. `b` still
   follows the selection. The driven note becomes a tooltip on the `g` row.
5. **Units** — one muted line under `b`, in the `.release-note` idiom, rather
   than eighteen tooltips or a unit in the legend.
6. **Trails** — `Style` unlabelled and full width, `Lines`, `Clear` at the head
   of the length row with the slider shortened to fit.

Two things to look at in the browser rather than on paper:

- The pendulum figures are a horizontal line with a filled circle on it, now
  sitting directly under two real sliders. Close up they separate — the bobs
  carry a white ring, the pivot is a dot, the slider thumb is bare accent —
  but row A, being a single, is the worst case.
- The Trails length label moving to the right of its slider is the one row on
  the panel that does not lead with its label.
