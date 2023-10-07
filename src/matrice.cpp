#include "utils.h"

/****************************************/
/**** CONSTANTES ****/
/****************************************/

// Pour faire avancer le robot
const int PULSES_PAR_TOUR = 3200;
const float VITESSE_AVANCER_MIN = 0.05;
const float VITESSE_AVANCER_MAX = 0.5;
const float CIRCONFERENCE_ROUE_M = 0.239389;
const float TAILLE_CELLULE = 0.475; // En metre

// Pour faire tourner le robot
const int PULSES_TOURNER_90_DEG = 1920;
const float VITESSE_TOURNER_MIN = 0.05;
const float VITESSE_TOURNER_MAX = 0.3;

// Pour les ajustement des vitesses des moteurs
const int INTERVALLE_PRISE_MESURE = 100; // En ms
const float PPM_TAUX_AJUSTEMENT_DISTANCE = 0.25; // Taux d'ajustement pour egaliser la distance parcourue totale des deux roues
const float CORRECTION_MIN = 0.9;
const float CORRECTION_MAX = 1.1;

// Dimensions du parcours
const int NB_COLS = 3;
const int NB_LIGNES = 10;

// Pins du robot
const int PIN_PROX_DROITE = 47; // Lumiere verte
const int PIN_PROX_GAUCHE = 49; // Lumiere rouge
const int PIN_MICRO = 0;


/****************************************/
/**** VARIABLES GLOBALES ****/
/****************************************/

int g_ligne;                        // Position y du robot
int g_colonne;                      // Position x du robot
int g_dir;                          // Direction cardinale du robot (0 = Nord, 1 = Est, 2 = Ouest, 3 = Sud)
int g_matrice[NB_COLS][NB_LIGNES];  // Matrice des cellules du parcours (0 = Case non exploree, 1 = Case exploree)


/****************************************/
/**** FONCTIONS (DECLARATIONS) ****/
/****************************************/

void prochaineCellule(int entryDir);
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

int dirOpposee(int dir);


/****************************************/
/**** SETUP & LOOP ****/
/****************************************/

void setup() {
  BoardInit();
}

// Test recursif
void loop2() {
  g_ligne = NB_LIGNES - 1;
  g_colonne = 1;
  g_dir = 0;
  g_matrice[NB_COLS][NB_LIGNES] = {0};

  // while (!sifflet()); // On attend le signal du sifflet

  // Fonction recursive qui va determiner le chemin a prendre et faire deplacer le robot
  prochaineCellule(-1);

  if (arrive())
    succes();
  else
    erreur(5);
}


/****************************************/
/**** FONCTIONS (DEFINITIONS) ****/
/****************************************/

// Cherche la prochaine cellule, appele recursivement
// Prend en parametre la direction d'entree dans la cellule. Si -1, premiere fois que fonction appelee
void prochaineCellule(int entryDir) {
  for (int dir=0; dir <= 3 && !arrive(); dir++)
    if (verifierCase(dir)) {
      deplacerCellule(dir);
      prochaineCellule(dir); // Si la case est non exploree et il n'y a pas de mur, on s'y deplace
    }

  if (arrive())
    return;

  if (entryDir != -1)
    deplacerCellule(dirOpposee(entryDir));
}

// Avance d'une certaine distance en metres
void avanceDistance(float distance){
  // Converti la distance en metre au nb de pulse necessaire
  const int PULSES_A_PARCOURIR = PULSES_PAR_TOUR * (distance / CIRCONFERENCE_ROUE_M);
  const int PPM_VOULU_MIN = 20;       // Distance min a parcourir par mesure pour correction

  float vitesseG = 0; // La vitesse de base des moteurs (sans correction)
  float vitesseD = 0;
  int encodeurG = 0; // Lit le nombre de pulses des encodeurs
  int encodeurD = 0;
  int distG = 0; // Distance totale en pulses parcourue par la roue
  int distD = 0;
  int PPMvoulu = 0; // Moyenne des 2 roues (Pulses par mesure)
  int PPMdiff = 0; // Difference des distances totales parcourues par les deux roues
  float correctionG = 1; // Multiplicateur correctif de la vitesse du robot pour que les deux roues roulent a la meme vitesse
  float correctionD = 1;

  // Tant que le nombre de pulse necessaire pour faire la distance desire est plus petit que le compteur des encoders
  while(distG < PULSES_A_PARCOURIR || distD < PULSES_A_PARCOURIR)
  {
    reinitialiserEncodeurs();

    int x = (distG + distD) / 2; // Distance parcourue jusqu'a present
    vitesseG = vitesseD = (VITESSE_AVANCER_MAX - VITESSE_AVANCER_MIN) * sin((PI*x)/PULSES_A_PARCOURIR) + VITESSE_AVANCER_MIN;

    MOTOR_SetSpeed(LEFT, vitesseG * correctionG);
    MOTOR_SetSpeed(RIGHT, vitesseD * correctionD);
    delay(INTERVALLE_PRISE_MESURE);

    distG += encodeurG = ENCODER_Read(LEFT);   // Update du nb de pulse
    distD += encodeurD = ENCODER_Read(RIGHT);

    PPMvoulu = (encodeurG + encodeurD) / 2;
    PPMdiff = (distG - distD) / PPM_TAUX_AJUSTEMENT_DISTANCE;

    // Ajuste la vitesse de chaque roue individuellement en fonction de la difference
    if (encodeurG > 0 && encodeurD > 0 && PPMvoulu > PPM_VOULU_MIN) {
      correctionG += (float)PPMvoulu / (encodeurG + PPMdiff) - 1;
      correctionD += (float)PPMvoulu / (encodeurD - PPMdiff) - 1;
    }

    // S'assurer que l'element de correction ne depasse pas les limites
    correctionG = minmax(CORRECTION_MIN, correctionG, CORRECTION_MAX);
    correctionD = minmax(CORRECTION_MIN, correctionD, CORRECTION_MAX);
  }

  arret();
}

// Fait tourner le robot de 90deg a gauche (LEFT) ou droite (RIGHT)
void tourne(int dir){
  const int PPM_VOULU_MIN = 20; // Distance min a parcourir par mesure pour correction

  float vitesseA = 0; // La vitesse de base des moteurs (sans correction)
  float vitesseR = 0;
  int encodeurA = 0; // Lit le nombre de pulses des encodeurs
  int encodeurR = 0;
  int distA = 0; // Distance totale en pulses parcourue par la roue
  int distR = 0;
  int PPMvoulu = 0; // Moyenne des 2 roues (Pulses par mesure)
  int PPMdiff = 0; // Difference des distances totales parcourues par les deux roues
  float correctionA = 1; // Multiplicateur correctif de la vitesse du robot pour que les deux roues roulent a la meme vitesse
  float correctionR = 1;
  int roueQuiAvance = -1;

  switch (dir) {
    case LEFT: roueQuiAvance = RIGHT; break;
    case RIGHT: roueQuiAvance = LEFT; break;
    default: erreur(6); // On quitte le programme, direction invalide
  }

  while(distA < PULSES_TOURNER_90_DEG || distR > -PULSES_TOURNER_90_DEG)
  {
    reinitialiserEncodeurs();

    int x = (distA - distR) / 2; // Distance parcourue jusqu'a present
    vitesseA = (VITESSE_TOURNER_MAX - VITESSE_TOURNER_MIN) * sin((PI*x)/PULSES_TOURNER_90_DEG) + VITESSE_TOURNER_MIN;
    vitesseR = -vitesseA;

    MOTOR_SetSpeed(roueQuiAvance, vitesseA * correctionA);
    MOTOR_SetSpeed(!roueQuiAvance, vitesseR * correctionR);
    delay(INTERVALLE_PRISE_MESURE);

    distA += encodeurA = ENCODER_Read(roueQuiAvance);   //update du nb de pulse
    distR += encodeurR = ENCODER_Read(!roueQuiAvance);

    PPMvoulu = (encodeurA - encodeurR) / 2;
    PPMdiff = (distA + distR) * PPM_TAUX_AJUSTEMENT_DISTANCE;

        // Ajuste la vitesse de chaque roue individuellement en fonction de la difference
    if (encodeurA > 0 && encodeurR < 0 && PPMvoulu > PPM_VOULU_MIN) {
      correctionA += (float)PPMvoulu / (encodeurA + PPMdiff) - 1;
      correctionR += (float)PPMvoulu / (-encodeurR - PPMdiff) - 1;
    }

    // S'assurer que l'element de correction ne depasse pas les limites
    correctionA = minmax(CORRECTION_MIN, correctionA, CORRECTION_MAX);
    correctionR = minmax(CORRECTION_MIN, correctionR, CORRECTION_MAX);
  }

  arret();
}

// Fait tourner le robot jusqu'a ce qu'il fait face a la bonne direction
void changerDirection(int dir) {
  while (g_dir != dir) {
    tourne(RIGHT);
    switch (g_dir) {
      case 0: g_dir = 1; break; // Nord  -> Est
      case 1: g_dir = 3; break; // Est   -> Sud
      case 2: g_dir = 0; break; // Ouest -> Nord
      case 3: g_dir = 2; break; // Sud   -> Ouest
    }
  }
}

// Deplace le robot vers la cellule devant celui-ci
void deplacerCellule(int dir) {
  const float DISTANCE_A_PARCOURIR = (dir == 0 || dir == 3) && g_ligne > 1
                                   ? TAILLE_CELLULE * 2
                                   : TAILLE_CELLULE;

  changerDirection(dir);
  avanceDistance(DISTANCE_A_PARCOURIR);
  switch (g_dir) {
    case 0: g_ligne--;    break;
    case 1: g_colonne++;  break;
    case 2: g_colonne--;  break;
    case 3: g_ligne++;    break;
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
  delay(100);
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

// Quitte le programme et emet le signal d'erreur
void erreur(int beepCount) {
  beep(beepCount, 100); // Erreur!
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
  return analogRead(PIN_MICRO) > 500;
}

// Detecte si le robot est arrivee a la derniere rangee
bool arrive() {
  return g_ligne == NB_LIGNES - 1;
}

// Verifie si la case dans la direction voulue existe et n'est pas exploree
bool verifierCase(int dir) {
  bool caseValide;

  // On verifie d'abord que la cellule existe et qu'elle est non exploree
  switch (dir) {
    case 0:   caseValide = g_ligne != 0             && g_matrice[g_colonne][g_ligne-1] == 0;
    case 1:   caseValide = g_colonne != NB_COLS - 1 && g_matrice[g_colonne+1][g_ligne] == 0;
    case 2:   caseValide = g_colonne != 0           && g_matrice[g_colonne-1][g_ligne] == 0;
    case 3:   caseValide = g_ligne != NB_LIGNES - 1 && g_matrice[g_colonne][g_ligne+1] == 0;
    default:  return false;
  }

  if (caseValide) {
    // On regarde ensuite s'il y a un mur dans le chemin
    changerDirection(dir);
    caseValide = !mur();
  }

  return caseValide;
}

// Retourne la direction opposee a celle mise en parametre
int dirOpposee(int dir) {
  switch (dir) {
    case 0: return 3; // Nord  -> Sud
    case 1: return 2; // Est   -> Ouest
    case 2: return 1; // Ouest -> Est
    case 3: return 0; // Sud   -> Nord
    default: return -1; // Invalide
  }
}
