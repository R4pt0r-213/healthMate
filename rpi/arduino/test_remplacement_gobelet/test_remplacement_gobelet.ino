#include <Servo.h>

// Commande attendue :
// angleVide;distanceVide;angleD;distanceD;angleS;distanceS;angleW;distanceW

#define NOMBRE_PARAMETRES_CYCLE 8

enum ServoId {
  BASE = 0,
  EPAULE = 1,
  COUDE = 2,
  POIGNET = 3,
  PINCE = 4
};

const byte NB_SERVOS = 5;

const byte SERVO_PINS[NB_SERVOS] = {9, 6, 5, 3, 11};

const int ANGLE_MIN[NB_SERVOS] = {0, 5, 20, 10, 30};
const int ANGLE_MAX[NB_SERVOS] = {180, 175, 175, 170, 180};

const int REPOS[NB_SERVOS] = {180, 63, 40, 50, 160};

const int PINCE_OUVERTE = 50;
const int PINCE_FERMEE = 170;

const int POIGNET_DEPOT_APPROCHE = 60;
const int POIGNET_VERTICAL = 75;

const int LIMITE_ZONE_PROCHE = 13;

Servo servos[NB_SERVOS];

float positions[NB_SERVOS] = {
  REPOS[BASE],
  REPOS[EPAULE],
  REPOS[COUDE],
  REPOS[POIGNET],
  REPOS[PINCE]
};

// Épaule, coude et pince restent alimentés pour maintenir le bras.
bool maintenirServo[NB_SERVOS] = {
  false,  // Base
  true,   // Épaule
  true,   // Coude
  false,  // Poignet
  true    // Pince
};

int limiterAngle(byte id, int angle) {
  return constrain(angle, ANGLE_MIN[id], ANGLE_MAX[id]);
}

float smoothstep(float t) {
  return t * t * (3.0f - 2.0f * t);
}

void attacherServo(byte id) {
  if (!servos[id].attached()) {
    servos[id].attach(SERVO_PINS[id]);
    servos[id].write((int)positions[id]);
    delay(20);
  }
}

void detacherServo(byte id) {
  if (!maintenirServo[id] && servos[id].attached()) {
    servos[id].detach();
  }
}

// Déplacement simultané des servos.
// Une cible égale à -1 signifie que le servo ne bouge pas.
void deplacerRobot(
  int cibleBase,
  int cibleEpaule,
  int cibleCoude,
  int ciblePoignet,
  int ciblePince,
  unsigned long duree
) {
  int cibles[NB_SERVOS] = {
    cibleBase,
    cibleEpaule,
    cibleCoude,
    ciblePoignet,
    ciblePince
  };

  float departs[NB_SERVOS];
  bool doitBouger[NB_SERVOS];

  int plusGrandDeplacement = 0;

  for (byte i = 0; i < NB_SERVOS; i++) {
    departs[i] = positions[i];

    if (cibles[i] >= 0) {
      cibles[i] = limiterAngle(i, cibles[i]);

      int ecart = abs(cibles[i] - (int)departs[i]);
      doitBouger[i] = ecart > 0;

      plusGrandDeplacement = max(plusGrandDeplacement, ecart);

      if (doitBouger[i]) {
        attacherServo(i);
      }
    } else {
      doitBouger[i] = false;
    }
  }

  if (plusGrandDeplacement == 0) {
    return;
  }

  const unsigned long intervalle = 20;
  int nombreEtapes = max(1UL, duree / intervalle);

  for (int etape = 0; etape <= nombreEtapes; etape++) {
    float t = (float)etape / nombreEtapes;
    float progression = smoothstep(t);

    for (byte i = 0; i < NB_SERVOS; i++) {
      if (doitBouger[i]) {
        float position =
          departs[i] +
          (cibles[i] - departs[i]) * progression;

        servos[i].write((int)position);
      }
    }

    delay(intervalle);
  }

  for (byte i = 0; i < NB_SERVOS; i++) {
    if (doitBouger[i]) {
      positions[i] = cibles[i];
      servos[i].write(cibles[i]);
      detacherServo(i);
    }
  }
}

void deplacerRobotAuto(
  int cibleBase,
  int cibleEpaule,
  int cibleCoude,
  int ciblePoignet,
  int ciblePince,
  int vitesseMsParDegre = 12
) {
  int cibles[NB_SERVOS] = {
    cibleBase,
    cibleEpaule,
    cibleCoude,
    ciblePoignet,
    ciblePince
  };

  int deplacementMaximum = 0;

  for (byte i = 0; i < NB_SERVOS; i++) {
    if (cibles[i] >= 0) {
      int cible = limiterAngle(i, cibles[i]);

      deplacementMaximum = max(
        deplacementMaximum,
        abs(cible - (int)positions[i])
      );
    }
  }

  unsigned long duree =
    max(300UL, (unsigned long)deplacementMaximum * vitesseMsParDegre);

  deplacerRobot(
    cibleBase,
    cibleEpaule,
    cibleCoude,
    ciblePoignet,
    ciblePince,
    duree
  );
}

void ouvrirPince() {
  Serial.println("ETAPE:OUVRIR_PINCE");

  maintenirServo[PINCE] = true;

  deplacerRobotAuto(
    -1, -1, -1, -1,
    PINCE_OUVERTE,
    8
  );
}

void fermerPince() {
  Serial.println("ETAPE:FERMER_PINCE");

  maintenirServo[PINCE] = true;

  deplacerRobotAuto(
    -1, -1, -1, -1,
    PINCE_FERMEE,
    12
  );

  delay(500);
}

void ouvrirPinceTresDoucement(int angleFinal) {
  Serial.println("ETAPE:OUVERTURE_PINCE_DOUCE");

  attacherServo(PINCE);

  int depart = (int)positions[PINCE];
  angleFinal = limiterAngle(PINCE, angleFinal);

  const int nombreEtapes = 80;

  for (int i = 0; i <= nombreEtapes; i++) {
    float t = (float)i / nombreEtapes;
    float progression = smoothstep(t);

    int position =
      depart + (int)((angleFinal - depart) * progression);

    servos[PINCE].write(position);
    delay(35);
  }

  positions[PINCE] = angleFinal;
  servos[PINCE].write(angleFinal);
  delay(300);
}

// Position haute utilisée ENTRE les opérations.
// Ce n'est pas encore la position de repos complète.
void positionTransport() {
  Serial.println("ETAPE:POSITION_TRANSPORT");

  deplacerRobotAuto(
    -1,
    63,
    100,
    REPOS[POIGNET],
    -1,
    18
  );
}

// Le vrai retour au repos n'est appelé qu'à la fin du remplacement.
void positionRepos() {
  Serial.println("ETAPE:POSITION_REPOS");

  // Remonter d'abord sans tourner la base.
  deplacerRobotAuto(
    -1,
    REPOS[EPAULE],
    120,
    REPOS[POIGNET],
    -1,
    14
  );

  // Finir la remontée et faire revenir la base.
  deplacerRobotAuto(
    REPOS[BASE],
    REPOS[EPAULE],
    REPOS[COUDE],
    REPOS[POIGNET],
    REPOS[PINCE],
    14
  );
}

void prendreObjet(int angleBase, int distanceObjet) {
  Serial.println("ETAPE:DEBUT_PRISE");

  angleBase = limiterAngle(BASE, angleBase);

  ouvrirPince();

  Serial.println("ETAPE:ROTATION_BASE");

  deplacerRobotAuto(
    angleBase,
    -1, -1, -1, -1,
    10
  );

  if (distanceObjet < LIMITE_ZONE_PROCHE) {
    Serial.println("ETAPE:APPROCHE_PROCHE");

    deplacerRobotAuto(
      -1,
      71,
      125,
      REPOS[POIGNET],
      -1,
      18
    );

    deplacerRobotAuto(
      -1,
      40,
      130,
      -1,
      -1,
      20
    );
  } else {
    Serial.println("ETAPE:APPROCHE_ELOIGNEE");

    deplacerRobotAuto(
      -1,
      35,
      75,
      REPOS[POIGNET],
      -1,
      15
    );

    deplacerRobotAuto(
      -1,
      10,
      90,
      -1,
      -1,
      18
    );
  }

  delay(300);

  fermerPince();

  delay(400);

  positionTransport();

  Serial.println("ETAPE:OBJET_PRIS");
}

// Le paramètre retourRepos permet de choisir :
// false = rester dans le cycle de remplacement ;
// true  = revenir complètement au repos.
void deposerObjet(
  int angleBase,
  int distanceDepot,
  bool retourRepos
) {
  Serial.println("ETAPE:DEBUT_DEPOT");

  angleBase = limiterAngle(BASE, angleBase);

  deplacerRobotAuto(
    angleBase,
    -1, -1, -1, -1,
    12
  );

  if (distanceDepot < LIMITE_ZONE_PROCHE) {
    Serial.println("ETAPE:DEPOT_PROCHE");

    deplacerRobotAuto(
      -1,
      48,
      110,
      POIGNET_DEPOT_APPROCHE,
      -1,
      20
    );

    deplacerRobotAuto(
      -1,
      48,
      110,
      POIGNET_VERTICAL,
      -1,
      24
    );
  } else {
    Serial.println("ETAPE:DEPOT_ELOIGNE");

    deplacerRobotAuto(
      -1,
      38,
      90,
      POIGNET_DEPOT_APPROCHE,
      -1,
      20
    );

    deplacerRobotAuto(
      -1,
      38,
      110,
      POIGNET_VERTICAL,
      -1,
      24
    );
  }

  delay(600);

  Serial.println("ETAPE:LACHER_GOBELET");

  ouvrirPinceTresDoucement(105);
  delay(700);

  ouvrirPinceTresDoucement(70);
  delay(700);

  ouvrirPinceTresDoucement(PINCE_OUVERTE);
  delay(500);

  // Après le dépôt en D, le bras remonte seulement en position de transport.
  // Le retour complet au repos est interdit tant que S n'a pas été déplacé vers W.
  if (retourRepos) {
    positionRepos();
  } else {
    positionTransport();
  }

  Serial.println("ETAPE:DEPOT_TERMINE");
}

int lireCommande(int valeurs[8]) {
  if (!Serial.available()) {
    return 0;
  }

  String ligne = Serial.readStringUntil('\n');
  ligne.trim();

  if (ligne.length() == 0) {
    return -1;
  }

  // R;angle : rotation de la base uniquement pour permettre à la caméra
  // de mesurer ArUco 4 avant le lancement du cycle complet.
  if (ligne.startsWith("R;")) {
    String champAngle = ligne.substring(2);
    champAngle.trim();

    if (champAngle.length() == 0 || champAngle.indexOf(';') >= 0) {
      return -1;
    }

    int angle = champAngle.toInt();

    if (angle < ANGLE_MIN[BASE] || angle > ANGLE_MAX[BASE]) {
      return -1;
    }

    valeurs[0] = angle;
    return -2;
  }

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

    champ.trim();

    if (champ.length() == 0) {
      return -1;
    }

    valeurs[nombre++] = champ.toInt();
  }

  if (debut < ligne.length()) {
    return -1;
  }

  if (nombre != NOMBRE_PARAMETRES_CYCLE) {
    return -1;
  }

  for (int i = 0; i < nombre; i += 2) {
    int angle = valeurs[i];
    int distance = valeurs[i + 1];

    if (angle < 0 || angle > 180 || distance < 0) {
      return -1;
    }
  }

  return nombre;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  // Initialisation des positions réelles.
  for (byte i = 0; i < NB_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(REPOS[i]);
  }

  delay(1200);

  for (byte i = 0; i < NB_SERVOS; i++) {
    detacherServo(i);
  }

  Serial.println("PRET");
}

void loop() {
  int valeurs[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  int nombre = lireCommande(valeurs);

  if (nombre == -2) {
    Serial.println("ETAPE:ALIGNEMENT_BASE");
    deplacerRobotAuto(
      valeurs[0],
      -1, -1, -1, -1,
      12
    );
    Serial.println("ALIGNE");
  } else if (nombre == NOMBRE_PARAMETRES_CYCLE) {
    Serial.println("CYCLE:DEBUT_REMPLACEMENT");

    // 1. Prendre le gobelet vide.
    prendreObjet(valeurs[0], valeurs[1]);

    // 2. Poser le gobelet vide en D.
    // PAS de retour complet au repos ici.
    Serial.println("CYCLE:VIDE_VERS_D");
    deposerObjet(
      valeurs[2],
      valeurs[3],
      false
    );

    // 3. Prendre le gobelet présent en S.
    Serial.println("CYCLE:PRISE_EN_S");
    prendreObjet(
      valeurs[4],
      valeurs[5]
    );

    // 4. Poser ce gobelet en W.
    Serial.println("CYCLE:S_VERS_W");
    deposerObjet(
      valeurs[6],
      valeurs[7],
      false
    );

    // 5. Le remplacement est terminé : retour au repos autorisé.
    positionRepos();

    Serial.println("TERMINE");
  } else if (nombre < 0) {
    Serial.println("ERREUR");
  }

  delay(20);
}
