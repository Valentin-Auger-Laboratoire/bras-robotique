# Bras robotique à trois axes

🇫🇷 [Français](#français) · 🇬🇧 [English](#english)

**Essayer l'outil :** https://valentin-auger-laboratoire.github.io/portfolio/outils/bras-robotique/

---

## Français

### Ce que fait l'outil
Simulation d'un bras robotique plan à trois articulations (épaule, coude, poignet), vu de côté. Elle est écrite en HTML / CSS / JavaScript pur, sans bibliothèque.

**Piloter le bras**
- **Axe par axe** (cinématique directe) : un curseur par moteur.
- **Par la position de l'outil** (cinématique inverse) : on clique ou on glisse dans la zone, et le bras calcule ses angles.
  - **Formule exacte** : on place le poignet, puis on résout le triangle épaule-coude-poignet par la loi des cosinus. Les deux solutions, coude haut et coude bas, sont dessinées.
  - **Méthode numérique** : moindres carrés amortis (Levenberg-Marquardt) sur la jacobienne, avec respect des butées, et courbe de convergence.
- Espace de travail, butées articulaires, cibles hors d'atteinte signalées.
- **Manipulabilité** w = √det(J·Jᵀ) et son ellipse en bout de bras. Alerte près des singularités (bras tendu ou replié).

**Trajectoires**
- Mode **articulaire** : chaque moteur rejoint son angle final, synchronisé sur le plus lent, et l'outil décrit une courbe.
- Mode **cartésien** : l'outil suit la ligne droite, et les angles sont recalculés à chaque instant.
- Profil de vitesse **trapézoïdal** (ou triangulaire pour les petits déplacements), avec vitesse et accélération réglables.
- Trajets : prendre et poser, ligne droite, cercle, carré, suture en zigzag.
- Graphes des angles et des vitesses des moteurs. Les trajets impossibles (hors de portée, butées) sont refusés.

**Couples et précision**
- Couple statique à chaque moteur, selon les masses des segments, des moteurs et de la charge portée.
- Pire cas (bras tendu à l'horizontale) et couple à prévoir avec un coefficient de sécurité.
- Couple à l'épaule selon son angle.
- **Précision en bout de bras** selon la résolution des codeurs (n bits), au premier ordre : |J|·Δq.

### Fichiers
| Fichier | Rôle |
|---|---|
| `robot.js` | Moteur : cinématique directe et inverse, jacobienne, manipulabilité, espace de travail, profil trapézoïdal, planification, couples (testable avec Node) |
| `index.html` | Interface et animation, bilingue FR / EN |
| `tests/test.js` | Tests du moteur |

### Lancer les tests
```bash
node tests/test.js
```
Environ 32 000 vérifications :
- cinématique directe sur des cas calculés à la main ;
- **cinématique inverse exacte sur 5 000 poses tirées au hasard**, retrouvées à 10⁻¹² près, la pose de départ faisant toujours partie des deux solutions ;
- jacobienne comparée aux différences finies ;
- manipulabilité nulle aux singularités et maximale coude à 90° ;
- méthode numérique : plus de 95 % de convergence depuis un départ proche, échec signalé pour une cible inatteignable ;
- profil trapézoïdal (durée, symétrie, vitesse jamais dépassée) ;
- trajectoire cartésienne rectiligne, écart de la trajectoire articulaire ;
- couples comparés au calcul à la main.

### Limites
- Bras **plan** à trois axes : pas de rotation de la base ni de cinématique 3D.
- Couples **statiques** : les efforts dus à l'accélération (dynamique) ne sont pas calculés.
- La méthode numérique est **locale** : partie d'une position éloignée, elle peut s'arrêter sur une butée ou un minimum local (environ 60 % de réussite depuis une position quelconque). C'est pour ça que le bras part de sa position actuelle.
- Pas de détection de collision du bras avec lui-même ou avec la table.

### Utiliser en local
Ouvrir `index.html` dans un navigateur.

---

## English

### What the tool does
Simulation of a planar robot arm with three joints (shoulder, elbow, wrist), seen from the side, in plain HTML / CSS / JavaScript.

- **Drive**:
  - joint by joint (forward kinematics), or by tool position (inverse kinematics);
  - the exact solution (law of cosines, elbow up and elbow down), or the numerical one (damped least squares on the Jacobian, with joint limits);
  - workspace, manipulability ellipse, singularity warnings.
- **Trajectories**: joint-space or Cartesian (straight line), trapezoidal velocity profile, pick and place, line, circle, square, zigzag suture; joint angle and speed plots.
- **Torques and accuracy**:
  - static motor torques from link, motor and payload masses, with the worst case and a safety factor;
  - tool accuracy from encoder resolution.

### Tests
```bash
node tests/test.js
```
About 32,000 checks:
- exact inverse kinematics on 5,000 random poses;
- Jacobian versus finite differences;
- singularities;
- numerical convergence;
- trapezoidal profile;
- straight Cartesian paths;
- hand-computed torques.

### Limits
- Planar arm, static torques only.
- Local numerical solver.
- No collision detection.

---

← [Portfolio](https://valentin-auger-laboratoire.github.io/portfolio/#projets)
