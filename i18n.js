'use strict';

// Every user-visible string in the interface, in both languages. Nothing in the
// UI should be a literal in index.js — if a string reaches the user, it belongs
// here. Keys are `section.thing`.
//
// Braces mark placeholders filled in at runtime, e.g. {n} in flight.parabola.
// The two languages must agree on which placeholders a string uses; they may
// reorder them freely.
//
// Not translated, deliberately: symbols and units (L, m, g, b, θ, ω, kg, s, J,
// °, N·m·s, m/s²) read the same in both languages, and numbers keep a decimal
// point throughout rather than switching to the French comma, because the
// number inputs beside them are parsed by the browser as HTML number fields and
// would not accept a comma. A mix of the two conventions on one panel would be
// worse than one consistent convention.
const STRINGS = {
  en: {
    'page.title': 'Pendulum Simulator',

    'btn.start': 'Start',
    'btn.pause': 'Pause',
    'btn.reset': 'Reset',
    'btn.cancel': 'Cancel',
    'btn.continue': 'Continue',
    // Shown before anything is lost, by the controls that cannot be applied to a
    // run in progress. It names what goes rather than asking "are you sure",
    // which is a question nobody can answer without being told that. The
    // question is the title; the body is what answering it costs.
    'reset.title': 'Restart the run?',
    'reset.confirm': 'This restarts every pendulum from its initial conditions and clears the trails.\n\nThe run is paused while you decide, and the restart arrives paused too.',

    'flight.legend': 'Flight simulator',
    'flight.tip': 'Drive gravity from the parabolic flight profile instead of the manual control.',
    'flight.clamped': 'clamped',
    'flight.parabola': 'Parabola {n}',
    'flight.clock': 't = {u} s of {c}',
    'flight.driven': 'g is driven by the flight profile',

    'phase.baseline': 'Baseline',
    'phase.pullup': 'Pull-up ↗',
    'phase.hyper': 'Hypergravity',
    'phase.injection': 'Injection ↘',
    'phase.apesanteur': 'Apesanteur',
    'phase.pullout': 'Pull-out ↗',
    'phase.recovery': 'Recovery ↘',

    'release.label': 'Release',
    'release.tip': 'Point in the cycle at which the sculpture is let go. It is clamped at its initial condition until then.',
    'release.note': 'Runs from {start}, released at {when} in {where}',

    'shortcut.label': 'Shortcut 1g baseline',
    'shortcut.tip': 'Trims the 93 s of level flight after the second hypergravity phase down to 10 s. The simulation still runs in real time — there is simply less of it between parabolas.',

    // No placeholder: the box holds all three pendulums, and each row says
    // which one it is with its dot. The unit is said once here rather than
    // after each of the nine figures.
    'energy.legend': 'Energy',
    'energy.unit': '— J',
    'energy.potential': 'Potential',
    'energy.kinetic': 'Kinetic',
    'energy.total': 'Total',

    'arch.legend': 'Architecture',
    'arch.sub': '— live',
    // The switch has no label on the panel — the tabs above it are its label —
    // so this names it for a screen reader instead of being drawn.
    'arch.links': 'Links',
    'arch.off': 'Off',
    'arch.single': 'Single',
    'arch.double': 'Double',
    'arch.triple': 'Triple',
    'arch.links.tip': 'Whether the selected pendulum hangs from the pivot, and on how many linked rods: one is regular and predictable, two or three are chaotic. Off takes it off the pivot and keeps its settings. Changing it re-runs every pendulum from its initial conditions.',
    'arch.friction.tip': 'Viscous friction at every hinge of the selected pendulum, in N·m·s/rad. 0 is a frictionless ideal pendulum.',
    'arch.pendulum': 'Pendulum {n}',
    'arch.pendulum.off': 'Pendulum {n} — off',

    'preset.moon': 'Moon',
    'preset.mars': 'Mars',
    'preset.earth': 'Earth',
    'preset.jupiter': 'Jupiter',
    'preset.zero': 'Zero',

    'init.legend': 'Initial conditions',
    'init.sub': '— on Reset',
    'init.use': 'Use current state as initial',

    'trails.legend': 'Trails',
    'trails.trace': 'Trace bob {i} of pendulum {n}',
    'trails.style': 'Style',
    'trails.style.tip': 'Line joins the recorded points into the bare path. Dots leaves one mark per frame, unjoined: since the frames are evenly spaced in time, the spacing between the dots is the speed.',
    'trails.dots': 'Dots',
    'trails.line': 'Line',
    'trails.length': 'Length',
    'trails.clear': 'Clear trails',

    'export.legend': 'Export',
    'export.span': 'Span',
    'export.current': 'current trail — the flight simulator is off',
    'export.png': 'PNG',
    'export.svg': 'SVG',
    'export.tip': 'First and last second of the flight cycle to save, from the most recent pass that has finished. Nothing but the traces is drawn and the background is transparent; SVG keeps the parameters that produced the pattern, PNG does not. The span is recorded separately from the trail above, so it can be saved from the moment it has been flown, whatever the trail Length is set to.',
    'export.auto': 'Auto',
    'export.auto.tip': 'Save the span automatically every time the flight finishes another pass over it. The browser will ask once for permission to download more than one file.',
    'export.autoNext': 'auto · next at t = {s} s',
    'export.ready': 'ready · {n} points · cycle {k}',
    'export.readyNow': 'ready · {n} points',
    'export.short': 'too long to record ({n} s)',
    'export.expired': 'flown before the last clear · comes round at t = {s} s',
    'export.unflown': 'not flown yet',
    'export.empty': 'no trace in this span',
    'export.none': 'nothing traced',
    'export.badspan': 'the end must come after the start',

    'state.legend': 'State — pendulum {n}',
    'state.time': 'Time',
    'state.rate': 'Rate',
    'state.rate.tip': 'Frames actually delivered, averaged over the last second. 60 is the display refresh and the ceiling; a lower number means frames are being missed.',
    'state.cost': 'JS',
    'state.cost.tip': 'Time spent in the simulation and drawing code per frame — mean and worst of the last second. The frame budget at 60 fps is 16.7 ms, and this counts only our own work, not the compositing the browser does after it.'
  },

  fr: {
    'page.title': 'Simulateur de pendule',

    'btn.start': 'Démarrer',
    'btn.pause': 'Pause',
    'btn.reset': 'Réinitialiser',
    'btn.cancel': 'Annuler',
    'btn.continue': 'Continuer',
    'reset.title': 'Relancer la simulation ?',
    'reset.confirm': "Ceci relance tous les pendules depuis leurs conditions initiales et efface les traces.\n\nLa simulation est en pause le temps de décider, et repart elle aussi en pause.",

    'flight.legend': 'Simulateur de vol',
    'flight.tip': 'Piloter la gravité par le profil de vol parabolique plutôt que par la commande manuelle.',
    'flight.clamped': 'bridée',
    'flight.parabola': 'Parabole {n}',
    'flight.clock': 't = {u} s sur {c}',
    'flight.driven': 'g est piloté par le profil de vol',

    // Aviation terms as Novespace uses them: the pull-up into a parabola is a
    // ressource, the level flight between them a palier.
    'phase.baseline': 'Palier 1 g',
    'phase.pullup': 'Ressource ↗',
    'phase.hyper': 'Hypergravité',
    'phase.injection': 'Injection ↘',
    'phase.apesanteur': 'Apesanteur',
    'phase.pullout': 'Sortie ↗',
    'phase.recovery': 'Rétablissement ↘',

    'release.label': 'Largage',
    'release.tip': "Point du cycle auquel la sculpture est lâchée. Elle reste bridée dans sa condition initiale jusque-là.",
    'release.note': 'Démarre à {start}, lâchée à {when} en {where}',

    'shortcut.label': 'Raccourcir le palier 1 g',
    'shortcut.tip': "Réduit à 10 s les 93 s de vol stabilisé qui suivent la seconde phase d'hypergravité. La simulation tourne toujours en temps réel — il y en a simplement moins entre les paraboles.",

    'energy.legend': 'Énergie',
    'energy.unit': '— J',
    'energy.potential': 'Potentielle',
    'energy.kinetic': 'Cinétique',
    'energy.total': 'Total',

    'arch.legend': 'Architecture',
    'arch.sub': '— en direct',
    'arch.links': 'Segments',
    'arch.off': 'Aucun',
    'arch.single': 'Simple',
    'arch.double': 'Double',
    'arch.triple': 'Triple',
    'arch.links.tip': 'Si le pendule sélectionné est suspendu, et sur combien de segments articulés : un seul est régulier et prévisible, deux ou trois sont chaotiques. « Aucun » le retire du point de suspension en conservant tous ses réglages. Le modifier relance tous les pendules depuis leurs conditions initiales.',
    'arch.friction.tip': 'Frottement visqueux à chaque articulation du pendule sélectionné, en N·m·s/rad. 0 correspond au pendule idéal sans frottement.',
    'arch.pendulum': 'Pendule {n}',
    'arch.pendulum.off': 'Pendule {n} — retiré',

    'preset.moon': 'Lune',
    'preset.mars': 'Mars',
    'preset.earth': 'Terre',
    'preset.jupiter': 'Jupiter',
    'preset.zero': 'Zéro',

    'init.legend': 'Conditions initiales',
    'init.sub': '— à la réinitialisation',
    'init.use': "Utiliser l'état actuel comme initiale",

    'trails.legend': 'Traces',
    'trails.trace': 'Tracer la masse {i} du pendule {n}',
    'trails.style': 'Style',
    'trails.style.tip': "Ligne relie les points enregistrés pour ne montrer que la trajectoire. Points laisse une marque par image, sans les relier : les images étant régulièrement espacées dans le temps, l'écart entre les points donne la vitesse.",
    'trails.dots': 'Points',
    'trails.line': 'Ligne',
    'trails.length': 'Durée',
    'trails.clear': 'Effacer les traces',

    'export.legend': 'Exporter',
    'export.span': 'Plage',
    'export.current': 'trace actuelle — le simulateur de vol est éteint',
    'export.png': 'PNG',
    'export.svg': 'SVG',
    'export.tip': "Première et dernière seconde du cycle de vol à enregistrer, prises au dernier passage terminé. Rien d'autre que les traces n'est dessiné et le fond est transparent ; le SVG conserve les paramètres qui ont produit le motif, le PNG non. La plage est enregistrée à part de la trace ci-dessus : elle peut donc être sauvegardée dès qu'elle a été parcourue, quelle que soit la durée de trace.",
    'export.auto': 'Auto',
    'export.auto.tip': "Enregistrer la plage automatiquement à chaque fois que le vol la parcourt de nouveau. Le navigateur demandera une fois l'autorisation de télécharger plusieurs fichiers.",
    'export.autoNext': 'auto · prochain à t = {s} s',
    'export.ready': 'prêt · {n} points · cycle {k}',
    'export.readyNow': 'prêt · {n} points',
    'export.short': 'trop longue pour être enregistrée ({n} s)',
    'export.expired': 'parcourue avant le dernier effacement · revient à t = {s} s',
    'export.unflown': 'pas encore parcourue',
    'export.empty': 'aucune trace dans cette plage',
    'export.none': 'aucune trace activée',
    'export.badspan': 'la fin doit venir après le début',

    'state.legend': 'État — pendule {n}',
    'state.time': 'Temps',
    'state.rate': 'Cadence',
    'state.rate.tip': "Images réellement affichées, moyennées sur la dernière seconde. 60 est la fréquence de l'écran, donc le plafond ; en dessous, des images sont perdues.",
    'state.cost': 'JS',
    'state.cost.tip': "Temps passé dans le calcul et le tracé par image — moyenne et pire cas de la dernière seconde. Le budget à 60 images/s est de 16,7 ms, et ceci ne compte que notre propre travail, pas la composition que le navigateur fait ensuite."
  }
};

// The switch offers the language you are not in, so its tooltip is written in
// the language you would get by clicking it.
const LANG_TIP = { en: 'Switch to English', fr: 'Passer en français' };
