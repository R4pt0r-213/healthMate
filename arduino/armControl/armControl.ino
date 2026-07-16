#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ----------------------------------------------------
// Écran OLED
// -1 évite le conflit avec le bouton placé sur la pin 4
// ----------------------------------------------------
#define OLED_RESET -1
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

// ----------------------------------------------------
// Identification des servos
// ----------------------------------------------------
enum ServoId {
  BASE = 0,       // Rotation autour de Z
  EPAULE = 1,     // Premier bras
  COUDE = 2,      // Deuxième bras
  POIGNET = 3,    // Rotation du poignet
  PINCE = 4
};

const byte NB_SERVOS = 5;

const byte servoPins[NB_SERVOS] = {
  9, 6, 5, 3, 11
};

const byte buttonPin = 4;

// ----------------------------------------------------
// Limites mécaniques
// À ajuster après essais pour éviter les blocages
// ----------------------------------------------------
const int ANGLE_MIN[NB_SERVOS] = {
  0, 5, 20, 10, 20
};

const int ANGLE_MAX[NB_SERVOS] = {
  180, 175, 175, 170, 165
};

// Position de repos.
// La pince est ouverte à 20° pour éviter de démarrer fermée.
const int POSITION_REPOS[NB_SERVOS] = {
  180, 63, 40, 50, 20
};

const int PINCE_OUVERTE = 20;
const int PINCE_DEPOT = 50;
const int PINCE_FERMEE = 165;

// À ajuster entre 65 et 90 selon ton montage
const int POIGNET_VERTICAL = 75;

// Limite entre la zone proche et la zone éloignée
const int LIMITE_ZONE_PROCHE = 13;

Servo servos[NB_SERVOS];

// Position mémorisée de chaque servo
float positions[NB_SERVOS];

// Servos qui doivent rester alimentés pour maintenir le bras
bool maintenirServo[NB_SERVOS] = {
  false,  // Base
  true,   // Épaule
  true,   // Coude
  false,  // Poignet
  true    // Pince
};

// ----------------------------------------------------
// Affichage
// ----------------------------------------------------
void afficher(const String &message) {
  Serial.println(message);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(message);
  display.display();
}

// ----------------------------------------------------
// Outils
// ----------------------------------------------------
int limiterAngle(byte numeroServo, int angle) {
  return constrain(
    angle,
    ANGLE_MIN[numeroServo],
    ANGLE_MAX[numeroServo]
  );
}

void attacherServo(byte numeroServo) {
  if (!servos[numeroServo].attached()) {
    servos[numeroServo].attach(servoPins[numeroServo]);

    // Réécriture immédiate de la dernière position connue
    servos[numeroServo].write((int)positions[numeroServo]);
    delay(20);
  }
}

void detacherServo(byte numeroServo) {
  if (!maintenirServo[numeroServo]) {
    servos[numeroServo].detach();
  }
}

float smoothstep(float t) {
  // Accélération douce, puis ralentissement doux
  return t * t * (3.0f - 2.0f * t);
}

// ----------------------------------------------------
// Déplacement simultané des cinq servos
//
// Une valeur -1 signifie que le servo ne doit pas bouger.
// ----------------------------------------------------
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

  // Aucun mouvement nécessaire
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

// Durée calculée automatiquement selon le déplacement
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
    max(300, deplacementMaximum * vitesseMsParDegre);

  deplacerRobot(
    cibleBase,
    cibleEpaule,
    cibleCoude,
    ciblePoignet,
    ciblePince,
    duree
  );
}

// ----------------------------------------------------
// Gestion de la pince
// ----------------------------------------------------
void ouvrirPince() {
  afficher("Ouverture pince");

  maintenirServo[PINCE] = true;

  deplacerRobotAuto(
    -1, -1, -1, -1,
    PINCE_OUVERTE,
    8
  );
}

void ouvrirPartiellementPince() {
  afficher("Depot objet");

  deplacerRobotAuto(
    -1, -1, -1, -1,
    PINCE_DEPOT,
    10
  );
}

void ouvrirPinceTresDoucement(int angleFinal) {
  afficher("Ouverture douce");

  attacherServo(PINCE);

  int depart = positions[PINCE];
  angleFinal = limiterAngle(PINCE, angleFinal);

  const int nombreEtapes = 80;

  for (int i = 0; i <= nombreEtapes; i++) {
    float t = (float)i / nombreEtapes;

    // Mouvement très doux
    float progression = smoothstep(t);

    int position =
      depart + (angleFinal - depart) * progression;

    servos[PINCE].write(position);
    delay(35);
  }

  positions[PINCE] = angleFinal;
  servos[PINCE].write(angleFinal);

  delay(300);
}

void fermerPince() {
  afficher("Prise objet");

  maintenirServo[PINCE] = true;

  deplacerRobotAuto(
    -1, -1, -1, -1,
    PINCE_FERMEE,
    12
  );

  delay(500);
}

// ----------------------------------------------------
// Position de repos
// ----------------------------------------------------
void positionRepos() {
  afficher("Position repos");

  // Remonter d'abord le bras sans faire tourner la base
  deplacerRobotAuto(
    -1,
    POSITION_REPOS[EPAULE],
    120,
    POSITION_REPOS[POIGNET],
    -1,
    14
  );

  // Finir la remontée et tourner la base simultanément
  deplacerRobotAuto(
    POSITION_REPOS[BASE],
    POSITION_REPOS[EPAULE],
    POSITION_REPOS[COUDE],
    POSITION_REPOS[POIGNET],
    POSITION_REPOS[PINCE],
    14
  );
}

// ----------------------------------------------------
// Prise d'un objet
// ----------------------------------------------------
void prendreObjet(int angleBase, int distanceObjet) {
  afficher("Debut prise");

  angleBase = limiterAngle(BASE, angleBase);

  ouvrirPince();

  // Tourner la base lorsque le bras est encore relevé
  deplacerRobotAuto(
    angleBase,
    -1, -1, -1, -1,
    10
  );

  if (distanceObjet < LIMITE_ZONE_PROCHE) {
    afficher("Zone proche");

  // Le coude descend moins qu'avant
  deplacerRobotAuto(
    -1,
    71,
    125,   // Avant : 140
    POSITION_REPOS[POIGNET],
    -1,
    18
  );

  // Approche finale moins prononcée
  deplacerRobotAuto(
    -1,
    40,
    130,   // Avant : 145
    -1,
    -1,
    20
  );
  } else {
    afficher("Zone eloignee");

    // Approche intermédiaire pour éviter une descente brutale
    deplacerRobotAuto(
      -1,
      35,
      75,
      POSITION_REPOS[POIGNET],
      -1,
      15
    );

    // Descente finale
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

// Remontée partielle :
// bras 1 presque vertical
// bras 2 presque horizontal
deplacerRobotAuto(
  -1,
  63,    // Bras 1 vertical
  100,   // Bras 2 horizontal environ
  POSITION_REPOS[POIGNET],
  -1,
  18
);

afficher("Objet pris");
}

// ----------------------------------------------------
// Dépôt d'un objet
// ----------------------------------------------------
void deposerObjet(int angleBase, int distanceDepot) {
  afficher("Debut depot");

  angleBase = limiterAngle(BASE, angleBase);

  // Rotation de la base pendant que le bras est en hauteur
  deplacerRobotAuto(
    angleBase,
    -1,
    -1,
    -1,
    -1,
    12
  );

  /*
    Le bras descend seulement jusqu'à une position intermédiaire.
    Le gobelet reste encore assez haut.
  */
  if (distanceDepot < LIMITE_ZONE_PROCHE) {
    afficher("Depot proche");

    deplacerRobotAuto(
      -1,
      52,
      110,
      60,
      -1,
      20
    );

    // Redresser doucement la pince sans descendre davantage
    deplacerRobotAuto(
      -1,
      52,
      110,
      POIGNET_VERTICAL,
      -1,
      24
    );

  } else {
    afficher("Depot eloigne");

    deplacerRobotAuto(
      -1,
      38,
      90,
      60,
      -1,
      20
    );

    // Redresser doucement la pince en restant en hauteur
    deplacerRobotAuto(
      -1,
      38,
      110,
      POIGNET_VERTICAL,
      -1,
      24
    );
  }

  // Laisser le bras se stabiliser
  delay(600);

  /*
    La pince s'ouvre alors que le bras est encore assez haut.
    Première ouverture légère : le gobelet commence à pivoter.
  */
  ouvrirPinceTresDoucement(105);

  delay(700);

  // Deuxième ouverture : le gobelet finit de se verticaliser
  ouvrirPinceTresDoucement(70);

  delay(700);

  // Ouverture complète
  ouvrirPinceTresDoucement(PINCE_OUVERTE);

  delay(500);

  // Remonter immédiatement, sans nouvelle descente
  deplacerRobotAuto(
    -1,
    POSITION_REPOS[EPAULE],
    POSITION_REPOS[COUDE],
    POSITION_REPOS[POIGNET],
    PINCE_OUVERTE,
    18
  );

  afficher("Depot termine");
}

// ----------------------------------------------------
// Initialisation
// ----------------------------------------------------
void setup() {
  Serial.begin(9600);

  // Le bouton est relié entre la pin 4 et GND
  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A6, INPUT);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.clearDisplay();

  afficher("Demarrage");

  // Initialisation des positions mémorisées
  for (byte i = 0; i < NB_SERVOS; i++) {
    positions[i] = POSITION_REPOS[i];
    servos[i].attach(servoPins[i]);
    servos[i].write(POSITION_REPOS[i]);
  }

  delay(1500);

  // Détacher uniquement les servos qui n'ont pas besoin de tenir
  for (byte i = 0; i < NB_SERVOS; i++) {
    detacherServo(i);
  }

  positionRepos();
}

// ----------------------------------------------------
// Boucle principale
// ----------------------------------------------------
void loop() {
  int anglePrise = map(
    analogRead(A0),
    0, 1023,
    ANGLE_MIN[BASE],
    ANGLE_MAX[BASE]
  );

  int distancePrise = map(
    analogRead(A1),
    0, 1023,
    8, 20
  );
  Serial.println(distancePrise);

  // Exemple : zone de dépôt située 50° plus loin
  int angleDepot = constrain(
    anglePrise + 50,
    ANGLE_MIN[BASE],
    ANGLE_MAX[BASE]
  );

  const int distanceDepot = 17;

  if (digitalRead(buttonPin) == LOW) {
    delay(30); // Antirebond

    if (digitalRead(buttonPin) == LOW) {
      prendreObjet(anglePrise, distancePrise);

      delay(500);

      deposerObjet(angleDepot, distanceDepot);

      delay(500);

      positionRepos();

      // Attend que le bouton soit relâché
      while (digitalRead(buttonPin) == LOW) {
        delay(10);
      }
    }
  }

  delay(20);
}