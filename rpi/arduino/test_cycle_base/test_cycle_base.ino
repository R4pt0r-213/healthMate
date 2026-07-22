#include <Servo.h>

// Test provisoire du cycle IA avec le servo de base uniquement.
const byte PIN_SERVO_BASE = 9;
const int DELAI_PAR_DEGRE_MS = 20;

Servo servoBase;
int positionBase = 90;


void deplacerBaseDoucement(int angleCible) {
  angleCible = constrain(angleCible, 0, 180);

  servoBase.attach(PIN_SERVO_BASE);
  servoBase.write(positionBase);
  delay(300);

  int pas = angleCible >= positionBase ? 1 : -1;

  for (
    int angle = positionBase;
    angle != angleCible;
    angle += pas
  ) {
    servoBase.write(angle);
    delay(DELAI_PAR_DEGRE_MS);
  }

  servoBase.write(angleCible);
  positionBase = angleCible;
  delay(400);
  servoBase.detach();
}


void setup() {
  Serial.begin(115200);
  Serial.setTimeout(150);
  Serial.println("PRET");
}


void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  String message = Serial.readStringUntil('\n');
  message.trim();

  int valeurs[8];
  int debut = 0;

  for (int index = 0; index < 8; index++) {
    int separateur = message.indexOf(';', debut);

    if (index < 7 && separateur < 0) {
      Serial.println("ERREUR_FORMAT");
      return;
    }

    String morceau = index == 7
      ? message.substring(debut)
      : message.substring(debut, separateur);

    valeurs[index] = morceau.toInt();
    debut = separateur + 1;
  }

  int angleVide = constrain(valeurs[0], 0, 180);
  int distanceVide = valeurs[1];
  int angleDepot = constrain(valeurs[2], 0, 180);
  int distanceDepot = valeurs[3];
  int anglePlein = constrain(valeurs[4], 0, 180);
  int distancePlein = valeurs[5];
  int angleDestination = constrain(valeurs[6], 0, 180);
  int distanceDestination = valeurs[7];

  Serial.print("VIDE W : ");
  Serial.print(angleVide);
  Serial.print(" deg, ");
  Serial.println(distanceVide);

  Serial.print("DEPOT D : ");
  Serial.print(angleDepot);
  Serial.print(" deg, ");
  Serial.println(distanceDepot);

  Serial.print("PLEIN S : ");
  Serial.print(anglePlein);
  Serial.print(" deg, ");
  Serial.println(distancePlein);

  Serial.print("DESTINATION W : ");
  Serial.print(angleDestination);
  Serial.print(" deg, ");
  Serial.println(distanceDestination);

  Serial.println("1_VIDE_VERS_PINCE");
  Serial.flush();
  deplacerBaseDoucement(angleVide);

  delay(500);

  Serial.println("2_VIDE_VERS_DEPOT_D");
  Serial.flush();
  deplacerBaseDoucement(angleDepot);

  delay(500);

  Serial.println("3_PLEIN_VERS_PINCE");
  Serial.flush();
  deplacerBaseDoucement(anglePlein);

  delay(500);

  Serial.println("4_PLEIN_VERS_DESTINATION_W");
  Serial.flush();
  deplacerBaseDoucement(angleDestination);

  Serial.println("TERMINE");
}
