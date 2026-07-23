#include <Servo.h>

// Test provisoire IA/Python -> Arduino -> servo de base uniquement.
const byte PIN_SERVO_BASE = 9;
const int ANGLE_MIN_BASE = 0;
const int ANGLE_MAX_BASE = 180;
const int DELAI_PAR_DEGRE_MS = 20;

Servo servoBase;
int positionBase = 180;


void deplacerBaseDoucement(int angleCible) {
  angleCible = constrain(
    angleCible,
    ANGLE_MIN_BASE,
    ANGLE_MAX_BASE
  );

  Serial.println("ATTACHE_BASE");

  servoBase.attach(PIN_SERVO_BASE);
  servoBase.write(positionBase);
  delay(400);

  if (angleCible > positionBase) {
    for (int angle = positionBase; angle <= angleCible; angle++) {
      servoBase.write(angle);
      delay(DELAI_PAR_DEGRE_MS);
    }
  } else {
    for (int angle = positionBase; angle >= angleCible; angle--) {
      servoBase.write(angle);
      delay(DELAI_PAR_DEGRE_MS);
    }
  }

  positionBase = angleCible;

  delay(400);
  servoBase.detach();

  Serial.println("DETACHE_BASE");
}


void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  // Le servo reste détaché au démarrage pour éviter un pic de courant.
  Serial.println("PRET");
}


void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  // Format compatible avec Python : angle;distance
  String message = Serial.readStringUntil('\n');
  message.trim();

  if (message.length() == 0) {
    return;
  }

  Serial.print("RECU : [");
  Serial.print(message);
  Serial.println("]");

  int separateur = message.indexOf(';');

  if (separateur == -1) {
    Serial.println("ERREUR_FORMAT");
    return;
  }

  int angle = message.substring(0, separateur).toInt();
  int distance = message.substring(separateur + 1).toInt();

  angle = constrain(angle, ANGLE_MIN_BASE, ANGLE_MAX_BASE);

  Serial.print("ANGLE : ");
  Serial.println(angle);

  // La distance est affichée pour valider le protocole, mais elle n'est pas
  // utilisée dans ce test limité au servo de rotation de la base.
  Serial.print("DISTANCE : ");
  Serial.println(distance);

  Serial.println("DEBUT");
  Serial.flush();

  deplacerBaseDoucement(angle);

  Serial.println("TERMINE");
}
