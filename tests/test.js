// Tests du moteur du bras robotique : node tests/test.js
const assert = require('assert');
const R = require('../robot.js');
const DEG = R.DEG;
let n = 0;
const close = (a, b, tol, msg) => { n++; assert.ok(Math.abs(a - b) <= tol * (1 + Math.abs(b)), (msg || '') + ' : ' + a + ' ≠ ' + b); };
const arm = { L: [0.4, 0.32, 0.12], lim: [[-170 * DEG, 170 * DEG], [-150 * DEG, 150 * DEG], [-160 * DEG, 160 * DEG]] };
let seed = 11;
const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
const randQ = () => arm.lim.map(([a, b]) => a + rnd() * (b - a));

// 1. Cinématique directe : cas à la main
{
  const f = R.fk(arm, [0, 0, 0]);
  close(f.x, 0.84, 1e-15, 'bras tendu : x = L1 + L2 + L3'); close(f.y, 0, 1e-15, 'bras tendu : y = 0');
  const g = R.fk(arm, [90 * DEG, -90 * DEG, 0]);
  close(g.x, 0.44, 1e-12, 'épaule à 90°, coude à −90° : x'); close(g.y, 0.4, 1e-12, 'y');
}

// 2. Cinématique inverse analytique : 5 000 poses tirées au hasard, retrouvées exactement
for (let k = 0; k < 5000; k++) {
  const q = randQ(), f = R.fk(arm, q), sols = R.ikAnalytic(arm, f.x, f.y, f.phi);
  n++; assert.ok(sols, 'pose atteignable');
  sols.forEach((s) => {
    const g = R.fk(arm, s);
    close(Math.hypot(g.x - f.x, g.y - f.y), 0, 1e-12, 'position par CI');
    close(Math.abs(R.wrap(g.phi - f.phi)), 0, 1e-12, 'orientation par CI');
  });
  n++; assert.ok(sols.some((s) => s.every((v, i) => Math.abs(R.wrap(v - q[i])) < 1e-9)), 'la pose de départ fait partie des deux solutions');
}
n++; assert.strictEqual(R.ikAnalytic(arm, 2, 0, 0), null, 'cible trop loin : aucune solution');
// coude haut / coude bas
{
  const sols = R.ikAnalytic(arm, 0.5, 0.2, -30 * DEG), up = R.pickElbow(sols, 'up'), down = R.pickElbow(sols, 'down');
  n++; assert.ok(Math.sin(up[0]) > Math.sin(down[0]), 'le coude haut est plus haut');
}

// 3. Jacobienne comparée aux différences finies
for (let k = 0; k < 300; k++) {
  const q = randQ(), J = R.jacobian(arm, q), f = R.fk(arm, q), h = 1e-7;
  for (let j = 0; j < 3; j++) {
    const qq = q.slice(); qq[j] += h;
    const g = R.fk(arm, qq);
    close(J[0][j], (g.x - f.x) / h, 1e-5, '∂x/∂q' + j); close(J[1][j], (g.y - f.y) / h, 1e-5, '∂y/∂q' + j);
  }
}
// manipulabilité : nulle bras tendu ou replié (2 axes), maximale coude à 90° pour l1 = l2
{
  const a2 = { L: [0.5, 0.5, 0], lim: arm.lim };
  close(R.manipulability(a2, [0.3, 0, 0]).w, 0, 1e-8, 'singularité bras tendu');
  close(R.manipulability(a2, [0.3, Math.PI, 0]).w, 0, 1e-8, 'singularité bras replié');
  close(R.manipulability(a2, [0, 90 * DEG, 0]).w, 0.25, 1e-12, 'w = l1·l2·|sin q2| à 90°');
  let best = 0, at = 0;
  for (let d = 1; d < 180; d++) { const w = R.manipulability(a2, [0, d * DEG, 0]).w; if (w > best) { best = w; at = d; } }
  n++; assert.strictEqual(at, 90, 'manipulabilité maximale à 90°');
}

// 4. Cinématique inverse numérique (moindres carrés amortis)
{
  let ok = 0;
  for (let k = 0; k < 500; k++) {
    const q = randQ(), f = R.fk(arm, q);
    // départ proche de la solution (cas d'usage : le bras suit la souris), écart jusqu'à ±17° par axe
    const q0 = q.map((v) => v + (rnd() - 0.5) * 0.6);
    const r = R.ikNumeric(arm, f.x, f.y, f.phi, q0, { wOri: 0.3, iters: 300, okTol: 1e-8 });
    if (r.ok) ok++;
  }
  n++; assert.ok(ok / 500 > 0.95, 'convergence depuis un départ proche dans plus de 95 % des cas : ' + ok / 5 + ' %');
  const r = R.ikNumeric(arm, 0.5, 0.3, 0, [0.1, 0.1, 0.1]);
  n++; assert.ok(r.ok && r.iters < 30, 'convergence rapide (' + r.iters + ' itérations)');
  const far = R.ikNumeric(arm, 1.5, 0, 0, [0.1, 0.1, 0.1], { iters: 200 });
  n++; assert.ok(!far.ok, 'cible inatteignable : échec signalé');
}

// 5. Espace de travail et portée
{
  const [r0, r1] = R.reach(arm);
  close(r1, 0.84, 1e-15, 'portée max'); close(r0, 0, 1e-15, 'portée min (l1 < l2 + l3)');
  close(R.reach({ L: [0.5, 0.2, 0.1] })[0], 0.2, 1e-15, 'portée min avec un bras long');
  const ws = R.workspace(arm, 12);
  n++; assert.ok(ws.every(([x, y]) => Math.hypot(x, y) <= r1 + 1e-12), 'espace de travail dans la portée');
}

// 6. Profil trapézoïdal
{
  const p = R.trapezoid(1, 0.5, 1);
  close(p.T, 2.5, 1e-15, 'durée : 2 × 0,5 s d’accélération + 1,5 s de palier');
  close(p.s(p.T), 1, 1e-15, 'distance totale'); close(p.s(p.T / 2), 0.5, 1e-15, 'symétrie');
  close(p.v(1.5), 0.5, 1e-15, 'vitesse de palier');
  const tri = R.trapezoid(0.1, 1, 1);
  close(tri.T, 2 * Math.sqrt(0.1), 1e-15, 'profil triangulaire'); close(tri.vPeak, Math.sqrt(0.1), 1e-15, 'vitesse de pointe');
  // la vitesse ne dépasse jamais la limite, et s croît
  let prev = -1;
  for (let k = 0; k <= 300; k++) { const tt = (p.T * k) / 300; n++; assert.ok(p.v(tt) <= 0.5 + 1e-12 && p.s(tt) >= prev - 1e-15); prev = p.s(tt); }
}

// 7. Trajectoires
{
  const q0 = R.pickElbow(R.ikAnalytic(arm, 0.6, 0.1, -60 * DEG), 'up');
  const target = { x: 0.3, y: 0.4, phi: -60 * DEG };
  // cartésienne : l'outil reste sur le segment
  const c = R.planMove(arm, q0, target, { mode: 'cart', vC: 0.3, aC: 0.6 });
  for (let k = 0; k <= 50; k++) {
    const f = R.fk(arm, c.at((c.T * k) / 50));
    const u = ((f.x - 0.6) * (0.3 - 0.6) + (f.y - 0.1) * (0.4 - 0.1)) / (0.3 ** 2 + 0.3 ** 2);
    close(Math.hypot(f.x - (0.6 - 0.3 * u), f.y - (0.1 + 0.3 * u)), 0, 1e-9, 'trajectoire cartésienne rectiligne');
  }
  const fe = R.fk(arm, c.at(c.T));
  close(fe.x, 0.3, 1e-12, 'arrivée x'); close(fe.y, 0.4, 1e-12, 'arrivée y');
  // articulaire : arrivée exacte, mais l'outil s'écarte de la ligne droite
  const j = R.planMove(arm, q0, target, { mode: 'joint', vJ: 90 * DEG, aJ: 180 * DEG, elbow: 'up' });
  const fj = R.fk(arm, j.at(j.T));
  close(fj.x, 0.3, 1e-12, 'arrivée articulaire x');
  const mid = R.fk(arm, j.at(j.T / 2));
  n++; assert.ok(Math.hypot(mid.x - 0.45, mid.y - 0.25) > 0.005, 'en articulaire, l’outil ne suit pas la ligne droite');
  // une ligne qui passe hors de portée est refusée
  const bad = R.planMove(arm, q0, { x: -0.6, y: 0.1, phi: -60 * DEG }, { mode: 'cart', vC: 0.3, aC: 0.6 });
  n++; assert.ok(bad && bad.blocked !== undefined, 'ligne impossible détectée');
}

// 8. Couples statiques : comparaison avec le calcul à la main
{
  const a2 = { L: [1, 1, 0] };
  const tq = R.gravityTorques(a2, [0, 0, 0], [2, 2, 0], 1, [], 9.81);
  close(tq[0], 9.81 * (2 * 0.5 + 2 * 1.5 + 1 * 2), 1e-12, 'couple à l’épaule, bras horizontal');
  close(tq[1], 9.81 * (2 * 0.5 + 1 * 1), 1e-12, 'couple au coude');
  const v = R.gravityTorques(a2, [90 * DEG, 0, 0], [2, 2, 0], 1, [], 9.81);
  close(v[0], 0, 1e-12, 'bras vertical : couple nul');
  // le couple à l'épaule est maximal bras horizontal
  let best = 0;
  for (let d = -180; d <= 180; d++) best = Math.max(best, Math.abs(R.gravityTorques(arm, [d * DEG, 0, 0], [1, 1, 0.3], 0.5, [0, 0.4, 0.4])[0]));
  close(best, Math.abs(R.gravityTorques(arm, [0, 0, 0], [1, 1, 0.3], 0.5, [0, 0.4, 0.4])[0]), 1e-12, 'pire cas = horizontale');
}
// 9. Petit système linéaire
{
  const x = R.solve([[2, 1], [1, 3]], [3, 5]);
  close(x[0], 0.8, 1e-14, 'solve x'); close(x[1], 1.4, 1e-14, 'solve y');
}

console.log('OK : ' + n + ' vérifications passent');
