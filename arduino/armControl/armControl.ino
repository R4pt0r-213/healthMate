#include <Servo.h>

// OLED test.
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET     4
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

//Mode bas
//const int POSREPOS[5] = {180, 63, 156, 137, 90};

//Mode haut
const int POSREPOS[5] = {180, 63, 40, 50, 165};
const int nbServo = 5;

const int servoPins[nbServo] = {9, 6, 5, 3, 11};

const int buttonPin = 4;

bool holdServo[5] = {false,true,true,false,false}; //1 toujours actif pour garder le poids qu'il porte sans tomber

Servo servos[nbServo];

int servo2;
int servo1;
int pServo1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  Serial.println("Demarrage");

  // OLED test.
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextColor(WHITE);//Sets the font display color
  display.clearDisplay();//cls
  //Set the font size
  display.setTextSize(2);
  //Set the display location
  display.setCursor(0, 0);
  //String displayed
  display.println(F("Demarrage"));
  //Began to show
  display.display();

  sleep();
  delay(2000);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A6, INPUT);

}

int angle=180;
void loop() {
  // put your main code here, to run repeatedly:
  int valA0 = analogRead(A0);
  int valA1 = analogRead(A1);
  int valA2 = analogRead(A2);
  int valA3 = analogRead(A3);
  int valA4 = analogRead(A6);

  
  int newAngle = map(valA0, 0, 1023, 0, 180);
  int distance = map(valA1, 0, 1023, 0, 180);
  servo1 = map(valA2, 0, 1023, 0, 180);
  servo2 = map(valA3, 0, 1023, 0, 180);
  pServo1 = map(valA4, 0, 1023, 0, 180);

  if (digitalRead(buttonPin) == LOW){
    
    
    Serial.println(newAngle);
    print(String(newAngle));
    take(newAngle, 10);
    delay(5000);
    drop(newAngle+50, 17);
    openGrip();
  

  }
  delay(100);


}

void attach(int pos){
  servos[pos].attach(servoPins[pos]);
}

void detach(int pos){
  if(!holdServo[pos]){ //pour garder le poids qu'il porte sans tomber
    servos[pos].detach();
  }
}

void attachAll(){
  for (int i=0; i<nbServo; i++){
    attach(i);
  }
}
void detachAll(){
  for (int i=0; i<nbServo; i++){
    detach(i);
  }
}

void move(int servo, int angle) {

  attach(servo);

  int depart = servos[servo].read();

  for (float t = 0; t <= 1.0; t += 0.02) {
    // Fonction d'easing (smoothstep)
    float ease = t * t * (3 - 2 * t);

    int pos = depart + (angle - depart) * ease;
    servos[servo].write(pos);

    delay(20);   // Augmenter pour aller plus lentement
  }

  servos[servo].write(angle);

  delay(300);
  detach(servo);
}

void moveTogetherSlow(int servo1, int angle1, int servo2, int angle2) {
  attach(servo1);
  attach(servo2);

  int depart1 = servos[servo1].read();
  int depart2 = servos[servo2].read();

  int distance1 = abs(angle1 - depart1);
  int distance2 = abs(angle2 - depart2);

  // Nombre d'étapes basé sur le déplacement le plus grand
  int nbEtapes = max(distance1, distance2);

  for (int i = 0; i <= nbEtapes; i++) {
    int position1 = depart1 + ((angle1 - depart1) * i) / nbEtapes;
    int position2 = depart2 + ((angle2 - depart2) * i) / nbEtapes;

    servos[servo1].write(position1);
    servos[servo2].write(position2);

    delay(25); // Augmenter pour ralentir
  }

  detach(servo1);
  detach(servo2);
}

void sleep(){
  //Va en position repos
  print("Repos");
  moveTogetherSlow(0, POSREPOS[0], 1, POSREPOS[1]);
  moveTogetherSlow(2, POSREPOS[2], 3, POSREPOS[3]);
  move(4, POSREPOS[4]);
  
}

void take(int angleX, int dist){
  holdServo[2]=true;
  openGrip();
  move(0,angleX);

  if (dist<13){
  /*
  Avec recul
  Possibilité d'aller jusqu'à 10cm, après trop proche
  */
    moveTogetherSlow(1, 71, 2, 140);
    delay(2000);
    move(1, 40);
  }
  else{
    //moveTogetherSlow(1, 0, 2, 18);
    move(2, 90);
    move(1, 10);
  }
  grap();
  moveTogetherSlow(1, POSREPOS[1], 2, 120);

}

void drop(int angleX, int dist){
  Serial.println("Debut du depot");

  // On garde le poids pendant le mouvement
  holdServo[2] = true;

  // Aller au-dessus de la zone de depot
  move(0, angleX);

  // Descendre à la position de depot
  if (dist>=13){
  /*
  Avec recul
  Possibilité d'aller jusqu'à 10cm, après trop proche
  */
    move(1, 30);
    move(2, 100);
  }
  else{
    move(2, 180);
  }
  // Lacher l'objet
  openPartialGrip();

  delay(1000);

  // Remonter après avoir lâché
  move(4, 0);
  moveTogetherSlow(1, POSREPOS[1], 2, 120);
  openGrip();

  Serial.println("Depot termine");
}

void openGrip(){
  Serial.println("On deserre la pince");
  holdServo[4]= false;
  move(4, 20);
}
void openPartialGrip(){
  Serial.println("On deserre la pince");
  holdServo[4]= false;
  move(4, 50);
}
void grap(){
  Serial.println("On serre la pince");
  holdServo[4]= true;
  attach(4);
  servos[4].write(165);
  delay(2000);
}


void print(String message){
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(message);
  display.display();
}
