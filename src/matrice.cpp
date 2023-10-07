#include "utils.h"

#define ERREUR -1

// Pour faire avancer le robot
const int PULSES_PAR_TOUR = 3200;
const float VITESSE_AVANCER_MIN = 0.05;
const float VITESSE_AVANCER_MAX = 0.6;
const float CIRCONFERENCE_ROUE_M = 0.239389;
const float TAILLE_CELLULE = 0.475; // En mètre

// Pour faire tourner le robot
const int PULSES_TOURNER_90_DEG = 1920;
const float VITESSE_TOURNER_MIN = 0.05;
const float VITESSE_TOURNER_MAX = 0.3;

// Pour les ajustement des vitesses des moteurs
const float CORRECTION_MIN = 0.9;
const float CORRECTION_MAX = 1.1;

// Pour avancer et tourner
const int INTERVALLE_PRISE_MESURE = 50; // En ms

// Dimensions du parcours
const int NB_COLONNES = 3;
const int NB_LIGNES = 10;

// Pins du robot
const int PIN_PROX_DROITE = 47; // Lumière verte
const int PIN_PROX_GAUCHE = 49; // Lumière rouge
const int PIN_MICRO = 0;

bool mur();
bool sifflet();
bool arrive(int ligne);
void avanceDistance(float distance);
void arret();
void tourne(int dir);
void changerDirection(int& currDir, int newDir);
void beep(int count, int ms = 75);
void reinitialiser_encodeurs();
int ENCODER_Read_Ajuste(int id);

void setup() {
  BoardInit();
}

void loop() {
  int ligne = NB_LIGNES - 1; // On commence du bas du tableau, donc index de la dernière ligne
  int colonne = 1;
  int dir = 0; // 0 = Nord, 1 = Est, 2 = Sud, 3 = Ouest
  int matrice[NB_COLONNES][NB_LIGNES] = {0};

  // while (!sifflet()); // On attend le signal du sifflet

  matrice[colonne][ligne] = 1;

  while (!arrive(ligne)) 
  {
    // Nord
    if (ligne != 0 && matrice[colonne][ligne-1] == 0) {
      changerDirection(dir, 0);
      if (!mur()) {
        avanceDistance(TAILLE_CELLULE);
        ligne--;
        matrice[colonne][ligne] = 1;
        continue;
      }
    }
    
    // Est
    if (colonne != NB_COLONNES - 1 && matrice[colonne+1][ligne] == 0) {
      changerDirection(dir, 1);
      if (!mur()) {
        avanceDistance(TAILLE_CELLULE);
        colonne++;
        matrice[colonne][ligne] = 1;
        continue;
      }
    }
    
    // Ouest
    if (colonne != 0 && matrice[colonne-1][ligne] == 0) {
      changerDirection(dir, 3);
      if (!mur()) {
        avanceDistance(TAILLE_CELLULE);
        colonne--;
        matrice[colonne][ligne] = 1;
        continue;
      }
    }

    // Sud
    if (ligne != NB_LIGNES - 1 && matrice[colonne][ligne+1] == 0) {
      changerDirection(dir, 2);
      if (!mur()) {
        avanceDistance(TAILLE_CELLULE);
        ligne++;
        matrice[colonne][ligne] = 1;
        continue;
      }
    }
  }

  beep(1, 1000);
  exit(EXIT_SUCCESS);
}

/********************************************************/
// Fonctions
/********************************************************/

// Detecte s'il y a un mur devant le robot
bool mur() {
  return !digitalRead(PIN_PROX_GAUCHE) || !digitalRead(PIN_PROX_DROITE);
}

// Detecte le signal du sifflet
bool sifflet() {
  // Serial.print(analogRead(PIN_MICRO));
  // Serial.print('\n');
  return true;
  return analogRead(PIN_MICRO) > 400;
}

// Detecte si le robot est arrivee a la derniere rangee
bool arrive(int ligne) {
  return ligne == NB_LIGNES - 1; // TODO
}

// Avance d'une certaine distance en mètres
void avanceDistance(float distance)
{
  // Converti la distance en mètre au nb de pulse nécéssaire
  const int PULSES_A_PARCOURIR = PULSES_PAR_TOUR * (distance / CIRCONFERENCE_ROUE_M);
  const int PPM_VOULU_MIN = 20;       // Distance min a parcourir par mesure pour correction
  const float PPM_TAUX_AJUSTEMENT_DISTANCE = 4.0; // Taux d'ajustement pour egaliser la difference de distance parcourue entre les deux roues
                                                  // Plus le nombre est grand, plus l'ajustement est moindre

  float vitesseG = VITESSE_AVANCER_MAX;   // La vitesse de base des moteurs
  float vitesseD = VITESSE_AVANCER_MAX;
  int encodeurG = 0;                  // Lit le nombre de pulses des encodeurs
  int encodeurD = 0;
  int pulsesParcourusG = 0;           // Nombre total de pulses emits par les roues depuis le debut du mouvement
  int pulsesParcourusD = 0;
  int PPMvoulu = 0;                   // Pulses par mesure qu'on veut que les deux roues parcours (moyenne des 2 roues)
  int PPMdiff = 0;                    // Difference entre les distances parcourues des deux roues
  float correctionG = 1;              // Multiplicateur correctif de la vitesse du robot pour que les deux roues roulent a la meme vitesse
  float correctionD = 1;

  // Tant que le nombre de pulse néccéssaire pour faire la distance desire est plus petit que le compteur des encoders
  while(pulsesParcourusG < PULSES_A_PARCOURIR || pulsesParcourusD < PULSES_A_PARCOURIR)
  {
    reinitialiser_encodeurs();

    int x = (pulsesParcourusG + pulsesParcourusD) / 2; // Distance parcourue jusqu'a present
    vitesseG = vitesseD = (VITESSE_AVANCER_MAX - VITESSE_AVANCER_MIN) * sin((PI*x)/PULSES_A_PARCOURIR) + VITESSE_AVANCER_MIN;

    MOTOR_SetSpeed(0, vitesseG * correctionG);
    MOTOR_SetSpeed(1, vitesseD * correctionD);
    delay(INTERVALLE_PRISE_MESURE);

    pulsesParcourusG += encodeurG = ENCODER_Read(LEFT);   // Update du nb de pulse
    pulsesParcourusD += encodeurD = ENCODER_Read(RIGHT);

    PPMvoulu = (encodeurG + encodeurD) / 2;
    PPMdiff = (pulsesParcourusG - pulsesParcourusD) / PPM_TAUX_AJUSTEMENT_DISTANCE;

    // Ajuste la vitesse de chaque roue individuellement en fonction de la difference
    if (encodeurG > 0 && encodeurD > 0 && PPMvoulu > PPM_VOULU_MIN) {
      correctionG += (float)PPMvoulu / (encodeurG + PPMdiff) - 1;
      correctionD += (float)PPMvoulu / (encodeurD - PPMdiff) - 1;
    }

    // S'assurer que l'element de correction ne depasse pas les limites
    correctionG = minmax(CORRECTION_MIN, correctionG, CORRECTION_MAX);
    correctionD = minmax(CORRECTION_MIN, correctionD, CORRECTION_MAX);

    // Serial.print("\nSum G: ");
    // Serial.print(pulsesParcourusG);
    // Serial.print("\nSum D: ");
    // Serial.print(pulsesParcourusD);
    // Serial.print("\nEncodeur G: ");
    // Serial.print(encodeurG);
    // Serial.print("\nEncodeur D: ");
    // Serial.print(encodeurD);
    // Serial.print("\nPPMvoulu: ");
    // Serial.print(PPMvoulu);
    // Serial.print("\nPPMdiff: ");
    // Serial.print(PPMdiff);
    // Serial.print("\nCorrection G: ");
    // Serial.print(correctionG);
    // Serial.print("\nCorrection D: ");
    // Serial.print(correctionD);
    // Serial.print("\nEx Correction G: ");
    // Serial.print((float)PPMvoulu / (encodeurG));
    // Serial.print("\nEx Correction D: ");
    // Serial.print((float)PPMvoulu / (encodeurD));
    // Serial.print("\n--------------------");
  }

  arret();
  delay(500);
}

// Fait arreter le robot
void arret(){
  MOTOR_SetSpeed(RIGHT, 0);
  MOTOR_SetSpeed(LEFT, 0);
  delay(250);
}

// Fait tourner le robot de 90deg a gauche (LEFT) ou droite (RIGHT)
void tourne(int dir){
  const int PPM_VOULU_MIN = 20;       // Distance min a parcourir par mesure pour correction
  const float PPM_TAUX_AJUSTEMENT_DISTANCE = 3.5; // Taux d'ajustement pour egaliser la difference de distance parcourue entre les deux roues
                                                  // Plus le nombre est grand, plus l'ajustement est moindre
  int encodeurA = 0; // Avance
  int encodeurR = 0; // Recule
  int sumA = 0;
  int sumR = 0;
  float vitesseA = 0;
  float vitesseR = 0;
  float correctionA = 1;              // Multiplicateur correctif de la vitesse du robot pour que les deux roues roulent a la meme vitesse
  float correctionR = 1;

  int PPMdiff = 0;  // Difference entre les distances parcourues des deux roues
  int PPMvoulu = 0; // Pulse per measure
  int roueQuiAvance = ERREUR;

  if (dir == LEFT)        roueQuiAvance = RIGHT;
  else if (dir == RIGHT)  roueQuiAvance = LEFT;
  
  if (roueQuiAvance == ERREUR) {
    beep(5, 250);
    delay(500);
    return;
  }

  while(sumA < PULSES_TOURNER_90_DEG || sumR > -PULSES_TOURNER_90_DEG)
  {
    reinitialiser_encodeurs();

    int x = (sumA - sumR) / 2; // Distance parcourue jusqu'a present
    vitesseA = (VITESSE_TOURNER_MAX - VITESSE_TOURNER_MIN) * sin((PI*x)/PULSES_TOURNER_90_DEG) + VITESSE_TOURNER_MIN;
    vitesseR = -vitesseA;

    MOTOR_SetSpeed(roueQuiAvance, vitesseA * correctionA);
    MOTOR_SetSpeed(!roueQuiAvance, vitesseR * correctionR);
    delay(INTERVALLE_PRISE_MESURE);

    sumA += encodeurA = ENCODER_Read(roueQuiAvance);   //update du nb de pulse
    sumR += encodeurR = ENCODER_Read(!roueQuiAvance);

    PPMvoulu = (encodeurA - encodeurR) / 2;
    PPMdiff = (sumA + sumR) / PPM_TAUX_AJUSTEMENT_DISTANCE;

        // Ajuste la vitesse de chaque roue individuellement en fonction de la difference
    if (encodeurA > 0 && encodeurR < 0 && PPMvoulu > PPM_VOULU_MIN) {
      correctionA = (float)PPMvoulu / (encodeurA + PPMdiff);
      correctionR = (float)PPMvoulu / (-encodeurR - PPMdiff);
    }

    // S'assurer que l'element de correction ne depasse pas les limites
    correctionA = minmax(CORRECTION_MIN, correctionA, CORRECTION_MAX);
    correctionR = minmax(CORRECTION_MIN, correctionR, CORRECTION_MAX);

    // Serial.print("\nSum A: ");
    // Serial.print(sumA);
    // Serial.print("\nSum R: ");
    // Serial.print(sumR);
    // Serial.print("\nVitesse A: ");
    // Serial.print(vitesseA);
    // Serial.print("\nVitesse R: ");
    // Serial.print(vitesseR);
    // Serial.print("\nPPMVoulu: ");
    // Serial.print(PPMvoulu);
    // Serial.print("\nPPMdiff: ");
    // Serial.print(PPMdiff);
    // Serial.print("\nEncodeur A: ");
    // Serial.print(encodeurA);
    // Serial.print("\nEncodeur R: ");
    // Serial.print(encodeurR);
    // Serial.print("\nCorrection A: ");
    // Serial.print(correctionA);
    // Serial.print("\nCorrection R: ");
    // Serial.print(correctionR);
    // Serial.print("\nEx Correction A: ");
    // Serial.print(PPMvoulu / (encodeurA));
    // Serial.print("\nEx Correction R: ");
    // Serial.print(PPMvoulu / (-encodeurR));
    // Serial.print("\n--------------------");
  }


  arret();
  delay(500);
}

// Change la direction du robot. 0 = Nord, 1 = Est, 2 = Sud, 3 = Ouest
void changerDirection(int& currDir, int newDir) {
  while (currDir != newDir) {
      tourne(RIGHT);
      currDir = (currDir + 1) % 4;
  }
}

// Fait beeper le robot un certain nombre de fois
void beep(int count, int ms){
  for(int i=0;i<count;i++) {
    AX_BuzzerON();
    delay(ms);
    AX_BuzzerOFF();
    delay(ms);  
  }
}

void reinitialiser_encodeurs() {
  ENCODER_ReadReset(LEFT);
  ENCODER_ReadReset(RIGHT);
}
