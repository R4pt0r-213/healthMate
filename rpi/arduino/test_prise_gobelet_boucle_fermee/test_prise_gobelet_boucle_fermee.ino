#include <Servo.h>

// Version autonome : PRISE SIMPLE AVEC ALIGNEMENT VISUEL.
#ifndef NOMBRE_PARAMETRES_CYCLE
#define NOMBRE_PARAMETRES_CYCLE 2
#endif

enum ServoId { BASE = 0, EPAULE = 1, COUDE = 2, POIGNET = 3, PINCE = 4 };

const byte NB_SERVOS = 5;
const byte SERVO_PINS[NB_SERVOS] = {9, 6, 5, 3, 11};
const int ANGLE_MIN[NB_SERVOS] = {0, 5, 20, 10, 30};
const int ANGLE_MAX[NB_SERVOS] = {180, 175, 175, 170, 180};
const int REPOS[NB_SERVOS] = {180, 63, 40, 50, 160};

const int PINCE_OUVERTE = 50;
const int PINCE_DEPOT = 100;
const int PINCE_FERMEE = 170;
const int POIGNET_DEPOT_APPROCHE = 60;
const int POIGNET_VERTICAL = 75;
const int LIMITE_ZONE_PROCHE = 13;

Servo servos[NB_SERVOS];
int positions[NB_SERVOS] = {180, 63, 40, 50, 160};

// Les articulations qui portent le bras ou le gobelet doivent continuer à
// fournir du couple après leur mouvement. La base et le poignet peuvent être
// libérés afin de réduire la consommation.
const bool HOLD_SERVO[NB_SERVOS] = {
  false,  // Base
  true,   // Épaule
  true,   // Coude
  false,  // Poignet
  true    // Pince
};

int limiter(byte id, int angle) {
  return constrain(angle, ANGLE_MIN[id], ANGLE_MAX[id]);
}

float smoothstep(float t) {
  return t * t * (3.0f - 2.0f * t);
}

// Les servos sont attachés progressivement. Ceux marqués HOLD_SERVO restent
// ensuite attachés afin de maintenir le bras et le gobelet.
void bougerServo(byte id, int cible, int vitesseMsParDegre = 14) {
  cible = limiter(id, cible);
  int depart = positions[id];
  int ecart = abs(cible - depart);

  if (ecart == 0) return;

  servos[id].attach(SERVO_PINS[id]);
  servos[id].write(depart);
  delay(120);  // laisse l'alimentation se stabiliser

  int etapes = max(1, ecart);
  for (int i = 1; i <= etapes; i++) {
    float t = (float)i / (float)etapes;
    int angle = depart + (int)((cible - depart) * smoothstep(t));
    servos[id].write(angle);
    delay(vitesseMsParDegre);
  }

  servos[id].write(cible);
  positions[id] = cible;
  delay(180);

  if (!HOLD_SERVO[id]) {
    servos[id].detach();
  }

  delay(250);  // évite d'attacher immédiatement le servo suivant
}

void ouvrirPince() {
  Serial.println("ETAPE:OUVRIR_PINCE");
  bougerServo(PINCE, PINCE_OUVERTE, 12);
}

void fermerPince() {
  Serial.println("ETAPE:FERMER_PINCE");
  bougerServo(PINCE, PINCE_FERMEE, 16);
}

void ouvrirPinceTresDoucement(int angleFinal) {
  Serial.println("ETAPE:OUVERTURE_PINCE_DOUCE");
  bougerServo(PINCE, angleFinal, 35);
  delay(300);
}

void remonterBras() {
  Serial.println("ETAPE:REMONTEE");
  // Le coude remonte avant l'épaule pour éloigner la pince du plateau.
  bougerServo(COUDE, 100, 16);
  bougerServo(EPAULE, 63, 16);
  bougerServo(POIGNET, REPOS[POIGNET], 14);
}

void prendreObjet(int angleBase, int distanceObjet) {
  Serial.println("ETAPE:DEBUT_PRISE");
  ouvrirPince();

  Serial.println("ETAPE:ROTATION_BASE");
  bougerServo(BASE, angleBase, 14);

  // Chaque articulation bouge séparément afin de limiter le courant demandé.
  if (distanceObjet < LIMITE_ZONE_PROCHE) {
    Serial.println("ETAPE:APPROCHE_PROCHE");
    bougerServo(COUDE, 125, 18);
    bougerServo(EPAULE, 71, 18);
    bougerServo(COUDE, 130, 18);
    bougerServo(EPAULE, 40, 20);
  } else {
    Serial.println("ETAPE:APPROCHE_ELOIGNEE");
    bougerServo(COUDE, 75, 16);
    bougerServo(EPAULE, 35, 16);
    bougerServo(COUDE, 90, 18);
    bougerServo(EPAULE, 10, 20);
  }

  delay(500);
  fermerPince();
  delay(600);
  remonterBras();
  Serial.println("ETAPE:OBJET_PRIS");
}

void deposerObjet(int angleBase, int distanceDepot) {
  Serial.println("ETAPE:DEBUT_DEPOT");

  // Le gobelet reste serré pendant la rotation et la descente.
  bougerServo(BASE, angleBase, 14);

  if (distanceDepot < LIMITE_ZONE_PROCHE) {
    Serial.println("ETAPE:DEPOT_PROCHE");
    bougerServo(COUDE, 110, 18);
    bougerServo(EPAULE, 52, 18);
    bougerServo(POIGNET, POIGNET_DEPOT_APPROCHE, 18);
    bougerServo(POIGNET, POIGNET_VERTICAL, 18);
  } else {
    Serial.println("ETAPE:DEPOT_ELOIGNE");
    bougerServo(COUDE, 90, 18);
    bougerServo(EPAULE, 38, 18);
    bougerServo(POIGNET, POIGNET_DEPOT_APPROCHE, 18);
    bougerServo(COUDE, 110, 18);
    bougerServo(POIGNET, POIGNET_VERTICAL, 18);
  }

  delay(600);
  Serial.println("ETAPE:LACHER_GOBELET");
  ouvrirPinceTresDoucement(105);
  delay(700);
  ouvrirPinceTresDoucement(70);
  delay(700);
  ouvrirPinceTresDoucement(PINCE_OUVERTE);
  delay(700);

  // La pince ne porte plus rien : elle n'a plus besoin de fournir du couple.
  servos[PINCE].detach();
  Serial.println("ETAPE:PINCE_DETACHEE");

  // On remonte avant de revenir à la géométrie de repos.
  //bougerServo(COUDE, 100, 18);
  bougerServo(EPAULE, REPOS[EPAULE], 18);
  bougerServo(COUDE, REPOS[COUDE], 18);
  bougerServo(POIGNET, REPOS[POIGNET], 14);
  Serial.println("ETAPE:DEPOT_TERMINE");
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  // Aucun attach au démarrage : aucune pointe de courant et aucun mouvement.
  for (byte i = 0; i < NB_SERVOS; i++) {
    servos[i].detach();
  }

  delay(500);
  Serial.println("PRET");
}

void loop() {
  if (!Serial.available()) return;

  String ligne = Serial.readStringUntil('\n');
  ligne.trim();

  // R;angle : rotation seule demandée par la boucle visuelle.
  if (ligne.startsWith("R;")) {
    int angle = ligne.substring(2).toInt();
    if (angle < 0 || angle > 180) {
      Serial.println("ERREUR");
      return;
    }

    Serial.println("ETAPE:ALIGNEMENT_BASE");
    bougerServo(BASE, angle, 14);
    Serial.println("ALIGNE");
    return;
  }

  // angle;distance : prise définitive après l'alignement (ou le repli ouvert).
  int separateur = ligne.indexOf(';');
  if (separateur <= 0 || separateur >= ligne.length() - 1) {
    Serial.println("ERREUR");
    return;
  }

  int angle = ligne.substring(0, separateur).toInt();
  int distance = ligne.substring(separateur + 1).toInt();
  if (angle < 0 || angle > 180 || distance < 0) {
    Serial.println("ERREUR");
    return;
  }

  Serial.print("RECU:");
  Serial.print(angle);
  Serial.print(';');
  Serial.println(distance);
  prendreObjet(angle, distance);
  Serial.println("TERMINE");
}
