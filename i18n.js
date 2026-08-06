'use strict';

// Every user-visible string, in both languages. If a string reaches the user it
// belongs here, not as a literal in index.js. Keys are `section.thing`.
//
// Braces mark placeholders filled at runtime, e.g. {n} in flight.parabola. The
// two languages must use the same placeholders, in any order.
//
// Not translated: symbols and units (L, m, g, b, θ, ω, kg, s, J, °, N·m·s,
// m/s²), and numbers keep a decimal point throughout, since the number inputs
// beside them are HTML number fields and would not accept a comma.
const STRINGS = {
  en: {
    'page.title': 'Pendulum Simulator',

    'btn.start': 'Start',
    'btn.pause': 'Pause',
    'btn.reset': 'Reset',
    'btn.cancel': 'Cancel',
    'btn.continue': 'Continue',
    // The theme switch carries no text, so its whole name is in its tooltip,
    // and it names the theme clicking gets you rather than the one you are in.
    'theme.dark': 'Switch to dark mode',
    'theme.light': 'Switch to light mode',
    // Shown by the controls that cannot be applied to a run in progress. The
    // title is the question; the body names what answering it costs.
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

    // The three terms, at the foot of each pendulum's Architecture row. They
    // name the figure beside them rather than heading a column, and each figure
    // carries its own J, since no legend is left to say it once.
    'energy.potential': 'Potential',
    'energy.kinetic': 'Kinetic',
    'energy.total': 'Total',

    'arch.legend': 'Architecture',
    'arch.sub': '— live',
    // Both step buttons name the link they move, so each is tied to the box it
    // will create or take away.
    'arch.add': 'add L{n}',
    'arch.remove': 'remove L{n}',
    // One line for all nine boxes below it. The placeholders are the units
    // themselves, picked out of the sentence in the text colour.
    'arch.units': 'Lengths in {L}, masses in {m}',
    'arch.length.tip': 'Length of this rod, in metres. The other rods take up the difference, so the pendulum keeps its reach; edit the total to change that.',
    'arch.total.tip': 'Pivot to tip, in metres. Changing it scales every rod of this pendulum together, so the shape of the chain is unchanged.',
    'arch.mass.tip': 'Mass of this bob, in kilograms. Its radius on the stage follows by volume.',
    'arch.friction.tip': 'Viscous friction at every hinge of the selected pendulum, in N·m·s/rad. 0 is a frictionless ideal pendulum.',
    'arch.pendulum': 'Pendulum {n} — on the pivot; click to take it off',
    'arch.pendulum.off': 'Pendulum {n} — off the pivot; click to hang it',
    'arch.pendulum.last': 'Pendulum {n} — the last one on the pivot, so it cannot be taken off',

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
    // The pair has no label on the panel — the two names are wide enough to be
    // their own — so this names it for a screen reader instead of being drawn.
    'trails.style': 'Style',
    'trails.style.tip': 'Lines joins the recorded points into the bare path. Dots leaves one mark per frame, unjoined: since the frames are evenly spaced in time, the spacing between the dots is the speed.',
    'trails.dots': 'Dots',
    'trails.line': 'Lines',
    'trails.length': 'Length',
    'trails.clear': 'Clear',

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

    // Names no pendulum: the box holds every one of them on the pivot, each
    // group led by its own letter.
    'state.legend': 'State',
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
    'theme.dark': 'Passer en mode sombre',
    'theme.light': 'Passer en mode clair',
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

    'energy.potential': 'Potentielle',
    'energy.kinetic': 'Cinétique',
    'energy.total': 'Total',

    'arch.legend': 'Architecture',
    'arch.sub': '— en direct',
    'arch.add': 'ajouter L{n}',
    'arch.remove': 'retirer L{n}',
    'arch.units': 'Longueurs en {L}, masses en {m}',
    'arch.length.tip': 'Longueur de ce segment, en mètres. Les autres segments absorbent la différence : le pendule conserve sa portée. Modifier le total pour la changer.',
    'arch.total.tip': "Du point de suspension à l'extrémité, en mètres. Le modifier met à l'échelle tous les segments de ce pendule ensemble : la forme de la chaîne ne change pas.",
    'arch.mass.tip': 'Masse de cette bille, en kilogrammes. Son rayon sur la scène en découle par le volume.',
    'arch.friction.tip': 'Frottement visqueux à chaque articulation du pendule sélectionné, en N·m·s/rad. 0 correspond au pendule idéal sans frottement.',
    'arch.pendulum': 'Pendule {n} — suspendu ; cliquer pour le retirer',
    'arch.pendulum.off': 'Pendule {n} — retiré ; cliquer pour le suspendre',
    'arch.pendulum.last': 'Pendule {n} — le dernier suspendu, il ne peut pas être retiré',

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
    'trails.style.tip': "Lignes relie les points enregistrés pour ne montrer que la trajectoire. Points laisse une marque par image, sans les relier : les images étant régulièrement espacées dans le temps, l'écart entre les points donne la vitesse.",
    'trails.dots': 'Points',
    'trails.line': 'Lignes',
    'trails.length': 'Durée',
    'trails.clear': 'Effacer',

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

    'state.legend': 'État',
    'state.time': 'Temps',
    'state.rate': 'Cadence',
    'state.rate.tip': "Images réellement affichées, moyennées sur la dernière seconde. 60 est la fréquence de l'écran, donc le plafond ; en dessous, des images sont perdues.",
    'state.cost': 'JS',
    'state.cost.tip': "Temps passé dans le calcul et le tracé par image — moyenne et pire cas de la dernière seconde. Le budget à 60 images/s est de 16,7 ms, et ceci ne compte que notre propre travail, pas la composition que le navigateur fait ensuite."
  }
};

// The switch offers the language you are not in, so each tooltip is written in
// the language clicking it would give you.
const LANG_TIP = { en: 'Switch to English', fr: 'Passer en français' };
