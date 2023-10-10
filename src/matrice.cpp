#include "utils.h"

/****************************************/
/**** CONSTANTES ****/
/****************************************/

// Pour faire avancer le robot
const int PULSES_PAR_TOUR = 3200;
const float VITESSE_AVANCER_MIN = 0.05;
const float VITESSE_AVANCER_MAX = 0.6;
const float CIRCONFERENCE_ROUE_M = 0.239389;
const float TAILLE_CELLULE = 0.49; // En metre

// Pour faire tourner le robot
const int PULSES_TOURNER_90_DEG = 1940;
const float VITESSE_TOURNER_MIN = 0.05;
const float VITESSE_TOURNER_MAX = 0.15;

// Pour les ajustement des vitesses des moteurs
const int INTERVALLE_PRISE_MESURE = 20; // En ms
const float CORRECTION_MIN = 0.8;
const float CORRECTION_MAX = 1.2;

// Dimensions du parcours
const int NB_COLS = 3;
const int NB_LIGNES = 6;

// Pour le micro
const int MICRO_VOLUME_START = 500;

// Pins du robot
const int PIN_PROX_DROITE = 47; // Lumiere verte
const int PIN_PROX_GAUCHE = 49; // Lumiere rouge
const int PIN_MICRO = PIN_A0;


/****************************************/
/**** ENUMS ****/
/****************************************/

enum Direction {
  NORD = 0,
  EST = 1,
  OUEST = 2,
  SUD = 3,
  AUCUNE_DIRECTION = 4
};


/****************************************/
/**** VARIABLES GLOBALES ****/
/****************************************/

int g_ligne;                        // Position y du robot
int g_colonne;                      // Position x du robot
int g_matrice[NB_COLS][NB_LIGNES];  // Matrice des cellules du parcours (0 = Case non exploree, 1 = Case exploree)
int g_dir;                          // Direction cardinale du robot


/****************************************/
/**** FONCTIONS (DECLARATIONS) ****/
/****************************************/

void avanceDistance(float distance);
void tourne(int dir);
void changerDirection(int dir);
void deplacerCellule(int dir);
void reinitialiserEncodeurs();
void arret();
void beep(int count, int ms);
void erreur(int beepCount);
void succes();

bool mur();
bool sifflet();
bool arrive();
bool verifierCase(int dir);

int prochaineDir(int leftright, int dir);
int directionOptimaleTourner(int dir);

float calculCorrection(int encodeurG, int encodeurD, int distG, int distD);


/****************************************/
/**** SETUP & LOOP ****/
/****************************************/

void setup() {
  BoardInit();
  pinMode(PIN_A0, INPUT);
}

void loop() {
  g_ligne = NB_LIGNES - 1;
  g_colonne = 1;
  g_dir = NORD;
  g_matrice[NB_COLS][NB_LIGNES] = {0};

  while (!sifflet()); // On attend le signal du sifflet

  while (!arrive())
    // On verifie chaque direction pour voir si on peut s'y deplacer
    for (int dir = NORD; dir <= AUCUNE_DIRECTION; dir++) {
      if (dir == AUCUNE_DIRECTION)
        erreur(5); // Erreur, aucune case valide, on quitte le programme
      if (verifierCase(dir)) {
        // Si la case est non exploree et il n'y a pas de mur, on s'y deplace
        // On avance tant qu'on peut (do while)
        do deplacerCellule(dir); while (verifierCase(dir)); 
        break; // On recommence la verification a partir de la direction Nord (0)
      }
    }

  succes();
}


/****************************************/
/**** FONCTIONS (DEFINITIONS) ****/
/****************************************/

// Avance d'une certaine distance en metres
void avanceDistance(float distance){
  // Converti la distance en metre au nb de pulse necessaire
  const int PULSES_A_PARCOURIR = PULSES_PAR_TOUR * (distance / CIRCONFERENCE_ROUE_M);

  float vitesseG = 0; // La vitesse de base des moteurs (sans correction)
  float vitesseD = 0;
  int encodeurG = 0; // Lit le nombre de pulses des encodeurs
  int encodeurD = 0;
  int distG = 0; // Distance totale en pulses parcourue par la roue
  int distD = 0;
  float correctionG = 1; // Multiplicateur correctif de la vitesse du robot pour que les deux roues roulent a la meme vitesse
  float correctionD = 1;

  reinitialiserEncodeurs();

  // Tant que le nombre de pulse necessaire pour faire la distance desire est plus petit que le compteur des encoders
  while(distG < PULSES_A_PARCOURIR || distD < PULSES_A_PARCOURIR)
  {
    const int X = (distG + distD) / 2; // Distance parcourue jusqu'a present
    const float VITESSE_BASE = (VITESSE_AVANCER_MAX - VITESSE_AVANCER_MIN) * 2 * sin((PI*X)/PULSES_A_PARCOURIR) + VITESSE_AVANCER_MIN;
    vitesseG = vitesseD = min(VITESSE_BASE, VITESSE_AVANCER_MAX);

    MOTOR_SetSpeed(LEFT, vitesseG * correctionG);
    MOTOR_SetSpeed(RIGHT, vitesseD * correctionD);
    delay(INTERVALLE_PRISE_MESURE);

    distG += encodeurG = ENCODER_ReadReset(LEFT); // Update du nb de pulse
    distD += encodeurD = ENCODER_ReadReset(RIGHT);

    // Calculer la correction a appliquer a la vitesse de base
    if (encodeurG > 0 && encodeurD > 0) {
      const float CORRECTION = calculCorrection(encodeurG, encodeurD, distG, distD);

      correctionG += CORRECTION;
      correctionD -= CORRECTION;
    }

    // S'assurer que l'element de correction ne depasse pas les limites
    correctionG = minmax(CORRECTION_MIN, correctionG, CORRECTION_MAX);
    correctionD = minmax(CORRECTION_MIN, correctionD, CORRECTION_MAX);
  }

  arret();
}

// Fait tourner le robot de 90deg a gauche (LEFT) ou droite (RIGHT)
void tourne(int dir){
  float vitesseA = 0; // La vitesse de base des moteurs (sans correction)
  float vitesseR = 0;
  int encodeurA = 0; // Lit le nombre de pulses des encodeurs
  int encodeurR = 0;
  int distA = 0; // Distance totale en pulses parcourue par la roue
  int distR = 0;
  float correctionA = 1; // Multiplicateur correctif de la vitesse du robot pour que les deux roues roulent a la meme vitesse
  float correctionR = 1;
  int roueQuiAvance = -1;

  switch (dir) {
    case LEFT:  roueQuiAvance = RIGHT;  break;
    case RIGHT: roueQuiAvance = LEFT;   break;
    default:    erreur(6); break; // On quitte le programme, direction invalide
  }

  reinitialiserEncodeurs();

  while(distA < PULSES_TOURNER_90_DEG || distR > -PULSES_TOURNER_90_DEG)
  {
    const int X = (distA - distR) / 2; // Distance parcourue jusqu'a present
    const float VITESSE_BASE = (VITESSE_TOURNER_MAX - VITESSE_TOURNER_MIN) * 2 * sin((PI*X)/PULSES_TOURNER_90_DEG) + VITESSE_TOURNER_MIN;
    vitesseA = min(VITESSE_BASE, VITESSE_TOURNER_MAX);
    vitesseR = -vitesseA;

    MOTOR_SetSpeed(roueQuiAvance, vitesseA * correctionA);
    MOTOR_SetSpeed(!roueQuiAvance, vitesseR * correctionR);
    delay(INTERVALLE_PRISE_MESURE);

    distA += encodeurA = ENCODER_ReadReset(roueQuiAvance); // Update du nb de pulse
    distR += encodeurR = ENCODER_ReadReset(!roueQuiAvance);

    // Calculer la correction a appliquer a la vitesse de base
    if (encodeurA > 0 && encodeurR < 0) {
      const float CORRECTION = calculCorrection(encodeurA, encodeurR, distA, distR);

      correctionA += CORRECTION;
      correctionR -= CORRECTION;
    }

    // S'assurer que l'element de correction ne depasse pas les limites
    correctionA = minmax(CORRECTION_MIN, correctionA, CORRECTION_MAX);
    correctionR = minmax(CORRECTION_MIN, correctionR, CORRECTION_MAX);
  }

  arret();
}

// Fait tourner le robot jusqu'a ce qu'il fait face a la bonne direction
void changerDirection(int dir) {
  int dirOptimale = directionOptimaleTourner(dir);
  while (g_dir != dir) {
    tourne(dirOptimale);
    g_dir = prochaineDir(dirOptimale, g_dir);
  }
}

// Deplace le robot vers la cellule devant celui-ci
void deplacerCellule(int dir) {
  const float DISTANCE_A_PARCOURIR = (dir == NORD || dir == SUD) && (g_ligne != 1 || dir != NORD)
                                   ? TAILLE_CELLULE * 2
                                   : TAILLE_CELLULE;

  changerDirection(dir);
  avanceDistance(DISTANCE_A_PARCOURIR);
  switch (g_dir) {
    case NORD:  g_ligne--;    break;
    case EST:   g_colonne++;  break;
    case OUEST: g_colonne--;  break;
    case SUD:   g_ligne++;    break;
    default:    erreur(8);    break; // On quitte le programme, direction invalide
  }
  g_matrice[g_colonne][g_ligne] = 1; // Marque la nouvelle case comme exploree
}

// Reinitialise les valeurs des encodeurs des moteurs
void reinitialiserEncodeurs() {
  ENCODER_ReadReset(LEFT);
  ENCODER_ReadReset(RIGHT);
}

// Fait arreter le robot
void arret(){
  MOTOR_SetSpeed(RIGHT, 0);
  MOTOR_SetSpeed(LEFT, 0);
}

// Fait beeper le robot un certain nombre de fois
void beep(int count, int ms){
  for(int i = 0; i < count; i++) {
    AX_BuzzerON();
    delay(ms);
    AX_BuzzerOFF();
    delay(ms);  
  }
}

// Quitte le programme et emet un signal d'erreur
void erreur(int beepCount) {
  beep(beepCount, 100);
  exit(EXIT_FAILURE);
}

// Quitte le programme et emet le signal de reussite du parcours
void succes() {
  beep(1, 1000);
  exit(EXIT_SUCCESS);
}

// Detecte s'il y a un mur devant le robot
bool mur() {
  return !digitalRead(PIN_PROX_GAUCHE) || !digitalRead(PIN_PROX_DROITE);
}

// Detecte le signal du sifflet
bool sifflet() {
  return analogRead(PIN_MICRO) > MICRO_VOLUME_START;
}

// Detecte si le robot est arrivee a la derniere rangee
bool arrive() {
  return g_ligne == 0;
}

// Verifie si la case dans la direction voulue existe et n'est pas exploree
bool verifierCase(int dir) {
  bool caseValide;

  // On verifie d'abord que la cellule existe et qu'elle est non exploree
  switch (dir) {
    case NORD:  caseValide = g_ligne != 0             && g_matrice[g_colonne][g_ligne-1] == 0; break;
    case EST:   caseValide = g_colonne != NB_COLS - 1 && g_matrice[g_colonne+1][g_ligne] == 0; break;
    case OUEST: caseValide = g_colonne != 0           && g_matrice[g_colonne-1][g_ligne] == 0; break;
    case SUD:   caseValide = g_ligne != NB_LIGNES - 1 && g_matrice[g_colonne][g_ligne+1] == 0; break;
    default:    erreur(9); break; // On quitte le programme, direction invalide
  }

  if (caseValide) {
    // On regarde ensuite s'il y a un mur dans le chemin
    changerDirection(dir);
    caseValide = !mur();
  }

  return caseValide;
}

// Determine vers quelle direction le robot fera face dependemment de la direction LEFT ou RIGHT
int prochaineDir(int leftright, int dir) {
  if (leftright == RIGHT) {
    switch (dir) {
      case NORD:  return EST;   break;
      case EST:   return SUD;   break;
      case SUD:   return OUEST; break;
      case OUEST: return NORD;  break;
    }
  }
  else if (leftright == LEFT) {
    switch (dir) {
      case NORD:  return OUEST; break;
      case OUEST: return SUD;   break;
      case SUD:   return EST;   break;
      case EST:   return NORD;  break;
    }
  }
  erreur(4);
  return AUCUNE_DIRECTION;
}

// Determine s'il est plus optimale pour le robot de tourner a gauche ou a droite pour faire face a la dir voulue
int directionOptimaleTourner(int dir) {
  int compteGauche = 0;
  int compteDroite = 0;
  int dirComparaison = 0;

  // On verifie combien de fois il faut tourner a droite
  dirComparaison = g_dir;
  while (dirComparaison != dir) {
    dirComparaison = prochaineDir(RIGHT, dirComparaison);
    compteDroite++;
  }

  // On verifie combien de fois il faut tourner a gauche
  dirComparaison = g_dir;
  while (dirComparaison != dir) {
    dirComparaison = prochaineDir(LEFT, dirComparaison);
    compteGauche++;
  }

  // On retourne la direction optimale
  return compteDroite <= compteGauche ? RIGHT : LEFT;
}

// Montant net a ajouter a l'element multiplicatif correctif pour les encodeurs
// La valeur de retour est additionnee au correctif G et soustrait au correctif D
float calculCorrection(int encodeurG, int encodeurD, int distG, int distD) {
  if (encodeurG == 0 || encodeurD == 0)
    return 0;

  // Le montant net est calcule avec les valeurs absolues des encodeurs, donc distance parcourue net
  encodeurG = abs(encodeurG);
  encodeurD = abs(encodeurD);
  distG = abs(distG);
  distD = abs(distD);

  const int AJU_DIST = distG > distD ? 1 : -1;
  const int DIFF_ENCO = encodeurG - encodeurD;
  const int DIFF_TOT = AJU_DIST + DIFF_ENCO;

  const float RAPPORT = (-(DIFF_TOT / 2.0) + encodeurG) / encodeurG;

  return (RAPPORT - 1) / 2;
}
