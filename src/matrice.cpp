#include <LibRobus.h>

#define ERREUR -1

// Pour faire avancer le robot
const int PULSES_PAR_TOUR = 3200;
const float VITESSE_MOTEURS = 0.2;
const float CIRCONFERENCE_ROUE_M = 0.239389;
const float TAILLE_CELLULE = 0.475; // En mètre

// Pour faire tourner le robot
const int PULSES_TOURNER_90_DEG = 1920;
const float VITESSE_TOURNER = 0.1;

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
void beep(int count, int ms = 75);
void reinitialiser_encodeurs();

void setup() {
  BoardInit();
}

void loop() {
  int ligne = NB_LIGNES - 1; // On commence du bas du tableau, donc index de la dernière ligne
  int colonne = 1;
  int dir = 0; // 0 = Nord, 1 = Est, 2 = Sud, 3 = Ouest
  int matrice[NB_COLONNES][NB_LIGNES] = {0};

  while (true) {
    tourne(RIGHT);
    tourne(LEFT);
  }

  // while (true) sifflet();

  // while (!sifflet()); // On attend le signal du sifflet

  matrice[colonne][ligne] = 1;

  while (!arrive(ligne)) 
  {
    // Nord
    if (ligne != 0 && matrice[colonne][ligne-1] == 0) {
      while (dir != 0) {
        tourne(RIGHT);
        dir = (dir + 1) % 4;
      }
      if (!mur()) {
        avanceDistance(TAILLE_CELLULE);
        ligne--;
        matrice[colonne][ligne] = 1;
        continue;
      }
    }
    
    // Est
    if (colonne != NB_COLONNES - 1 && matrice[colonne+1][ligne] == 0) {
      while (dir != 1) {
        tourne(RIGHT);
        dir = (dir + 1) % 4;
      }
      if (!mur()) {
        avanceDistance(TAILLE_CELLULE);
        colonne++;
        matrice[colonne][ligne] = 1;
        continue;
      }
    }
    
    // Ouest
    if (colonne != 0 && matrice[colonne-1][ligne] == 0) {
      while (dir != 3) {
        tourne(RIGHT);
        dir = (dir + 1) % 4;
      }
      if (!mur()) {
        avanceDistance(TAILLE_CELLULE);
        colonne--;
        matrice[colonne][ligne] = 1;
        continue;
      }
    }

    // Sud
    if (ligne != NB_LIGNES - 1 && matrice[colonne][ligne+1] == 0) {
      while (dir != 2) {
        tourne(RIGHT);
        dir = (dir + 1) % 4;
      }
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
  Serial.print(analogRead(PIN_MICRO));
  Serial.print('\n');
  return false;
  return analogRead(PIN_MICRO) > 400;
}

// Detecte si le robot est arrivee a la derniere rangee
bool arrive(int ligne) {
  return ligne == NB_LIGNES - 1; // TODO
}

// Avance d'une certaine distance en mètres
void avanceDistance(float distance)
{
  //Converti la distance en mètre au nb de pulse nécéssaire
  int nbPulseVoulu = PULSES_PAR_TOUR * (distance / CIRCONFERENCE_ROUE_M);
  
  //initialise la vitesse des moteurs
  float vitesseG = VITESSE_MOTEURS;
  float vitesseD = VITESSE_MOTEURS;

  int encodeurG = 0;
  int encodeurD = 0;
  int sumG = 0;
  int sumD = 0;
  int PPMvoulu = 0; // Pulse per measure

  //Tant que le nombre de pulse néccéssaire pour faire la distance désiré est plus petit que le compteur des encoders
  while(sumG < nbPulseVoulu || sumD < nbPulseVoulu)
  {
    reinitialiser_encodeurs();

    MOTOR_SetSpeed(0, vitesseG);
    MOTOR_SetSpeed(1, vitesseD);
    delay(INTERVALLE_PRISE_MESURE);

    sumG += encodeurG = ENCODER_Read(LEFT);   //update du nb de pulse
    sumD += encodeurD = ENCODER_Read(RIGHT);

    PPMvoulu = (encodeurG + encodeurD) / 2;

    //Ajuste la vitesse de chaque roue individuellement en fonction de la différence
    if (encodeurG > 0 && encodeurD > 0) {
      vitesseG = (PPMvoulu * vitesseG) / encodeurG;
      vitesseD = (PPMvoulu * vitesseD) / encodeurD;
    }
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

// Fait tourner le robot
void tourne(int dir){
  int encodeurA = 0; // Avance
  int encodeurR = 0; // Recule
  int sumA = 0;
  int sumR = 0;
  float vitesseA = VITESSE_TOURNER;
  float vitesseR = -VITESSE_TOURNER;

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

    MOTOR_SetSpeed(roueQuiAvance, vitesseA);
    MOTOR_SetSpeed(!roueQuiAvance, vitesseR);
    delay(INTERVALLE_PRISE_MESURE);

    sumA += encodeurA = ENCODER_Read(roueQuiAvance);   //update du nb de pulse
    sumR += encodeurR = ENCODER_Read(!roueQuiAvance);

    PPMvoulu = (encodeurA + -encodeurR) / 2;

    //Ajuste la vitesse de chaque roue individuellement en fonction de la différence
    if (encodeurA > 0 && encodeurR < 0) {
      vitesseA = (PPMvoulu * vitesseA) / encodeurA;
      vitesseR = (-PPMvoulu * vitesseR) / encodeurR;
    }
  }

  arret();
  delay(500);
}

// Fait changer de colonne le robot
void changer_colonne(int& colonne_actuelle) {
  arret();

  int prochaine_colonne = colonne_actuelle;
  bool changement_fini = false;

  // Faire changer le robot de colonne
  while (!changement_fini) {
    prochaine_colonne = (prochaine_colonne + 1) % 3;

    if (prochaine_colonne < colonne_actuelle) {
      tourne(LEFT);
      for (int i = 0; i < (colonne_actuelle - prochaine_colonne) && !mur(); i++) {
        avanceDistance(TAILLE_CELLULE);
        changement_fini = true;
      }
      tourne(RIGHT);
    }
    else if (prochaine_colonne > colonne_actuelle) {
      tourne(RIGHT);
      for (int i = 0; i < (prochaine_colonne - colonne_actuelle) && !mur(); i++) {
        avanceDistance(TAILLE_CELLULE);
        changement_fini = true;
      }
      tourne(LEFT);
    }
  }

  // Mettre a jour la colonne actuelle du robot
  colonne_actuelle = prochaine_colonne;
}

// Fait beeper le robot un certain nombre de fois
void beep(int count, int ms){
  for(int i=0;i<count;i++) {
    AX_BuzzerON();
    delay(ms);
    AX_BuzzerOFF();
    delay(ms);  
  }
  delay(400);
}

void reinitialiser_encodeurs() {
  ENCODER_ReadReset(LEFT);
  ENCODER_ReadReset(RIGHT);
}
