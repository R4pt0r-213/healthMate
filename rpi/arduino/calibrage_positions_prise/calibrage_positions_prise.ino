#include <Servo.h>

// Réglage manuel d'une trajectoire de prise.
//
// A0 à A4 pilotent respectivement les servos 0 à 4.
// Bouton entre l'entrée 4 et GND (INPUT_PULLUP) :
//   - appui court : mémorise une position intermédiaire ;
//   - maintien 4 s : mémorise la position finale et affiche le résultat.
//
// Le programme Python envoie la distance sous la forme : D;14.25

enum ServoId {
  BASE = 0,
  EPAULE = 1,
  COUDE = 2,
  POIGNET = 3,
  PINCE = 4
};

const byte NB_SERVOS = 5;
const byte SERVO_PINS[NB_SERVOS] = {9, 6, 5, 3, 11};
const byte POTENTIOMETRE_PINS[NB_SERVOS] = {A0, A1, A2, A3, A4};

const int ANGLE_MIN[NB_SERVOS] = {0, 5, 20, 10, 30};
const int ANGLE_MAX[NB_SERVOS] = {180, 175, 175, 170, 180};

const int buttonPin = 4;
const unsigned long DUREE_APPUI_FINAL_MS = 4000;
const unsigned long DEBOUNCE_MS = 40;
const int SEUIL_MOUVEMENT_DEG = 1;
const byte MAX_ETAPES = 16;

Servo servos[NB_SERVOS];
int positions[NB_SERVOS] = {90, 90, 90, 90, 90};

struct EtapePrise {
  float distanceCm;
  int angles[4];  // Servos 1, 2, 3 et 4 uniquement.
  bool finale;
};

EtapePrise etapes[MAX_ETAPES];
byte nombreEtapes = 0;

float distanceGobeletCm = -1.0;
bool boutonStable = HIGH;
bool derniereLectureBouton = HIGH;
bool appuiFinalEnregistre = false;
unsigned long changementBoutonMs = 0;
unsigned long debutAppuiMs = 0;
unsigned long dernierAffichageMs = 0;

int lireAnglePotentiometre(byte id) {
  int lecture = analogRead(POTENTIOMETRE_PINS[id]);
  return map(
    lecture,
    0,
    1023,
    ANGLE_MIN[id],
    ANGLE_MAX[id]
  );
}

void afficherEtape(const EtapePrise &etape, byte numero) {
  Serial.print("ETAPE ");
  Serial.print(numero);
  Serial.print(" | distance=");
  Serial.print(etape.distanceCm, 2);
  Serial.print(" cm | S1=");
  Serial.print(etape.angles[0]);
  Serial.print(" | S2=");
  Serial.print(etape.angles[1]);
  Serial.print(" | S3=");
  Serial.print(etape.angles[2]);
  Serial.print(" | S4=");
  Serial.print(etape.angles[3]);
  Serial.print(" | ");
  Serial.println(etape.finale ? "FINALE" : "INTERMEDIAIRE");
}

void afficherResultat() {
  Serial.println();
  Serial.println("--- RESULTAT A ENVOYER A CODEX ---");
  Serial.print("Nombre d'etapes : ");
  Serial.println(nombreEtapes);

  for (byte i = 0; i < nombreEtapes; i++) {
    afficherEtape(etapes[i], i + 1);
  }

  Serial.println("--- FIN DU RESULTAT ---");
  Serial.println();
}

void enregistrerEtape(bool finale) {
  if (distanceGobeletCm < 0.0) {
    Serial.println(
      "ERREUR : aucune distance reçue. Lance calibrerPositionsPrise.py."
    );
    return;
  }

  if (nombreEtapes >= MAX_ETAPES) {
    Serial.println("ERREUR : nombre maximal d'etapes atteint.");
    return;
  }

  EtapePrise &etape = etapes[nombreEtapes];
  etape.distanceCm = distanceGobeletCm;
  etape.finale = finale;

  // Ne pas enregistrer le servo 0 : il dépend de la direction du gobelet.
  for (byte id = EPAULE; id <= PINCE; id++) {
    etape.angles[id - 1] = positions[id];
  }

  nombreEtapes++;
  afficherEtape(etape, nombreEtapes);

  if (finale) {
    afficherResultat();
  }
}

void lireDistanceSerie() {
  if (!Serial.available()) {
    return;
  }

  String ligne = Serial.readStringUntil('\n');
  ligne.trim();

  if (ligne.startsWith("D;")) {
    float nouvelleDistance = ligne.substring(2).toFloat();

    if (nouvelleDistance > 0.0) {
      distanceGobeletCm = nouvelleDistance;
    }
  } else if (ligne == "RESET") {
    nombreEtapes = 0;
    distanceGobeletCm = -1.0;
    Serial.println("CALIBRATION REINITIALISEE");
  }
}

void mettreAJourServos() {
  for (byte id = 0; id < NB_SERVOS; id++) {
    int nouvellePosition = lireAnglePotentiometre(id);

    if (abs(nouvellePosition - positions[id]) >= SEUIL_MOUVEMENT_DEG) {
      positions[id] = nouvellePosition;
      servos[id].write(positions[id]);
    }
  }
}

void gererBouton() {
  bool lecture = digitalRead(buttonPin);
  unsigned long maintenant = millis();

  if (lecture != derniereLectureBouton) {
    changementBoutonMs = maintenant;
    derniereLectureBouton = lecture;
  }

  if (
    maintenant - changementBoutonMs >= DEBOUNCE_MS
    && lecture != boutonStable
  ) {
    boutonStable = lecture;

    if (boutonStable == LOW) {
      debutAppuiMs = maintenant;
      appuiFinalEnregistre = false;
    } else if (!appuiFinalEnregistre) {
      // Relâché avant quatre secondes : étape intermédiaire.
      enregistrerEtape(false);
    }
  }

  if (
    boutonStable == LOW
    && !appuiFinalEnregistre
    && maintenant - debutAppuiMs >= DUREE_APPUI_FINAL_MS
  ) {
    enregistrerEtape(true);
    appuiFinalEnregistre = true;
  }
}

void afficherEtatPeriodique() {
  if (millis() - dernierAffichageMs < 1000) {
    return;
  }

  dernierAffichageMs = millis();
  Serial.print("distance=");

  if (distanceGobeletCm < 0.0) {
    Serial.print("INCONNUE");
  } else {
    Serial.print(distanceGobeletCm, 2);
    Serial.print("cm");
  }

  Serial.print(" | S0=");
  Serial.print(positions[BASE]);
  Serial.print(" S1=");
  Serial.print(positions[EPAULE]);
  Serial.print(" S2=");
  Serial.print(positions[COUDE]);
  Serial.print(" S3=");
  Serial.print(positions[POIGNET]);
  Serial.print(" S4=");
  Serial.println(positions[PINCE]);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(30);
  pinMode(buttonPin, INPUT_PULLUP);

  // Lecture initiale des potentiomètres avant d'alimenter les servos.
  for (byte id = 0; id < NB_SERVOS; id++) {
    positions[id] = lireAnglePotentiometre(id);
  }

  // Évite un appel de courant simultané au démarrage.
  for (byte id = 0; id < NB_SERVOS; id++) {
    servos[id].attach(SERVO_PINS[id]);
    servos[id].write(positions[id]);
    delay(180);
  }

  Serial.println("PRET");
  Serial.println("A0..A4 : regler les servos 0..4");
  Serial.println("Appui court : etape intermediaire");
  Serial.println("Appui 4 secondes : etape finale et resultat");
}

void loop() {
  lireDistanceSerie();
  mettreAJourServos();
  gererBouton();
  afficherEtatPeriodique();
  delay(10);
}
