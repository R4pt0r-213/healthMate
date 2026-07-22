#include <Servo.h>

// Version autonome : PRISE PUIS DEPOT.
#define NOMBRE_PARAMETRES_CYCLE 4

enum ServoId { BASE = 0, EPAULE = 1, COUDE = 2, POIGNET = 3, PINCE = 4 };

const byte NB_SERVOS = 5;
const byte SERVO_PINS[NB_SERVOS] = {9, 6, 5, 3, 10};
const int ANGLE_MIN[NB_SERVOS] = {0, 5, 20, 10, 20};
const int ANGLE_MAX[NB_SERVOS] = {180, 175, 175, 170, 165};
const int REPOS[NB_SERVOS] = {180, 63, 40, 50, 20};

const int PINCE_OUVERTE = 50;
const int PINCE_FERMEE = 170;
const int LIMITE_ZONE_PROCHE = 13;

Servo servos[NB_SERVOS];
int positions[NB_SERVOS] = {180, 63, 40, 50, 20};

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
    bougerServo(POIGNET, 75, 18);
  } else {
    Serial.println("ETAPE:DEPOT_ELOIGNE");
    bougerServo(COUDE, 90, 18);
    bougerServo(EPAULE, 38, 18);
    bougerServo(COUDE, 110, 18);
    bougerServo(POIGNET, 75, 18);
  }

  delay(600);
  Serial.println("ETAPE:LACHER_GOBELET");
  ouvrirPince();
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

// Commandes acceptées :
// 2 valeurs : prise simple
// 4 valeurs : prise puis dépôt
// 8 valeurs : vide -> D, puis plein équivalent -> ancienne place du vide
int lireCommande(int valeurs[8]) {
  if (!Serial.available()) return false;

  String ligne = Serial.readStringUntil('\n');
  ligne.trim();

  int debut = 0;
  int nombre = 0;
  while (debut < ligne.length() && nombre < 8) {
    int separateur = ligne.indexOf(';', debut);
    String champ;
    if (separateur == -1) {
      champ = ligne.substring(debut);
      debut = ligne.length();
    } else {
      champ = ligne.substring(debut, separateur);
      debut = separateur + 1;
    }
    if (champ.length() == 0) return 0;
    valeurs[nombre++] = champ.toInt();
  }

  // Une donnée supplémentaire signifie que la commande est mal formée.
  if (debut < ligne.length()) return -1;
  if (nombre != NOMBRE_PARAMETRES_CYCLE) return -1;
  for (int i = 0; i < nombre; i += 2) {
    if (valeurs[i] < 0 || valeurs[i] > 180 || valeurs[i + 1] < 0) return -1;
  }
  return nombre;
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
  int valeurs[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  int nombre = lireCommande(valeurs);

  if (nombre == NOMBRE_PARAMETRES_CYCLE) {
    Serial.print("RECU:");
    Serial.print(valeurs[0]);
    Serial.print(';');
    Serial.println(valeurs[1]);

    prendreObjet(valeurs[0], valeurs[1]);
    if (NOMBRE_PARAMETRES_CYCLE == 4) {
      deposerObjet(valeurs[2], valeurs[3]);
    }

    if (NOMBRE_PARAMETRES_CYCLE == 8) {
      Serial.println("CYCLE:VIDE_VERS_DEPOT");
      deposerObjet(valeurs[2], valeurs[3]);

      Serial.println("CYCLE:PRISE_GOBELET_PLEIN");
      prendreObjet(valeurs[4], valeurs[5]);

      Serial.println("CYCLE:PLEIN_VERS_ANCIENNE_PLACE");
      deposerObjet(valeurs[6], valeurs[7]);
    }

    Serial.println("TERMINE");
  } else if (nombre < 0) {
    Serial.println("ERREUR");
  }
}
