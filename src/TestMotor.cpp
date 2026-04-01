#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "event_groups.h"
#include <AccelStepper.h>

// Code pour tester les branchement des deux moteurs, teste effectuer le 2026-03-11 par Cloé et Marco B.
// Résulta : les moteurs fonctionnent mais les courois skipe des dents

//BOUTONS
#define BTN_PIN_UP 22
#define BTN_PIN_DOWN 25
#define BTN_PIN_LEFT 23
#define BTN_PIN_RIGHT 24
#define BTN_PIN_OK 26
#define BTN_Interup 3
#define DEBOUNCE_MS 20

//LIMIT SWITCHES
#define PIN_LMSW_INTER 2 //D2
#define LMTSW_Y 38
#define LMTSW_X 40
EventGroupHandle_t limitEventGroup;
#define EVT_LIMIT_X (1 << 0)
#define EVT_LIMIT_Y (1 << 1)

//MOTEURS
#define FULLSTEP 4 // 4 fils par moteurs
#define STEPS_REV 4096

#define SerialGripper    Serial1 //A CHANGER SELON PORTS

//Moteur 1
#define EN_PIN_M1 29
#define DIR_PIN_M1 27
#define STEP_PIN_M1 28
#define SW_RX_M1            12 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX_M1            11 // TMC2208/TMC2224 SoftwareSerial transmit pin

//Moteur 2
#define EN_PIN_M2 32
#define DIR_PIN_M2 30
#define STEP_PIN_M2 31
#define SW_RX_M2            10 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX_M2            9 // TMC2208/TMC2224 SoftwareSerial transmit pin

//Moteur Z
#define EN_PIN_MZ 35
#define DIR_PIN_MZ 33
#define STEP_PIN_MZ 34
#define SW_RX_MZ            14 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX_MZ            13 // TMC2208/TMC2224 SoftwareSerial transmit pin

AccelStepper MOT_A = AccelStepper(AccelStepper::DRIVER, STEP_PIN_M1,DIR_PIN_M1); //Moteur gauche
AccelStepper MOT_B = AccelStepper(AccelStepper::DRIVER, STEP_PIN_M2,DIR_PIN_M2); //Moteur droite
AccelStepper MOT_Z = AccelStepper(AccelStepper::DRIVER, STEP_PIN_MZ,DIR_PIN_MZ); //Moteur Z

enum SystemState {
  IDLE,
  LOWERING, //On descend l'axe Z
  CLOSING, //Pince fermée
  LIFTING, //Remonter Axe Z
  MOVING_TO_DROPZONE, //Se déplacer vers la zone de dépôt
  DROPPING, //Pince ouverte
};
volatile SystemState currentState = IDLE;
bool toutouAttrape = false; // Variable globale pour stocker l'état du toutou attrapé

EventGroupHandle_t inputEventGroup;
EventGroupHandle_t toutouEventGroup;
TaskHandle_t motorTaskHandle = NULL;

float targetA = 0;
float targetB = 0;

int deltaX = 0;
int deltaY = 0;

long posX = 6550;
long posY = 0;

float speed = 1000; // Vitesse commence à bouger -> 26

long MAX_POS_X = 13100;
long MAX_POS_Y = 12998;

// Ajout pour test
#define BTN_OK 0
#define BTN_UP 1
#define BTN_DOWN 2
#define BTN_LEFT 3
#define BTN_RIGHT 4
int bouton_direction_tester = 0;

/*
1. Test déplacement chariot X selon une direction 
(2026-03-25 12h) Cloé
UP -> diago (bon sens)
DOWN -> pas bouger
LEFT -> diago (bon sens)
RIGHT -> pas bouger

mise en commentaire du code de limite
(2026-03-25 14h 30) Cloé
UP -> 
DOWN -> diago (bon sens)
LEFT -> 
RIGHT ->

raison pourquoi déplacement diago : le moteur poulie haut tourne mais pas la poulie, Amélie a reviser vise poulie
UP -> Ok
DOWN -> Ok
LEFT -> Ok
RIGHT -> OK

2. Test limite de position
(2026-03-25 15h) Cloé
50000 -> trop loin
selon possition imprimer : 12870


1.2 Retester déplacement dans une direction toute direction
(2026-03-26 15h) Cloé
UP -> Ok
DOWN -> Ok
LEFT -> Ok
RIGHT -> OK

*** mettre les boutons pour controler machine -> faciliter test -> bouton pas fonctionner***

2.2 Tester déplacement combiner
(2026-03-26 15h) Cloé
UP DOWN-> OK
LEFT RIGHT -> Ok
UP LEFT -> Ok
UP RIGHT -> OK
DOWN LEFT -> OK
DOWN RIGHT ->OK
UP DOWN LEFT -> OK
UP DOWN LEFT RIGHT-> Ok

3.2 Déterminer limite de position
(2026-03-26 17h) Cloé
Pour vitesse de 2
X -> 13100
Y -> 12998

4.2 Code permet empécher aller or limite
(2026-03-26 18h) Cloé
-> pas réussi arrêter moteur dans if

5. Test bouton pour déplacer core xy direction précise
(2026-04-01) -À Cloé
Les mvt corespond au bouton appuyer
Lors relacher bouton décélération trop lente

6. Modifier programme pour fonctionner en vitesse


*/

void setup() {
  Serial.begin(115200);
  SerialGripper.begin(115200);
  delay(2000);

  //MOTEURS
  //----------------
  //Moteur 1
  pinMode(EN_PIN_M1, OUTPUT);
  pinMode(STEP_PIN_M1, OUTPUT);
  pinMode(DIR_PIN_M1, OUTPUT);
  digitalWrite(EN_PIN_M1, LOW);

  //Moteur 2
  pinMode(EN_PIN_M2, OUTPUT);
  pinMode(STEP_PIN_M2, OUTPUT);
  pinMode(DIR_PIN_M2, OUTPUT);
  digitalWrite(EN_PIN_M2, LOW);

  //Moteur Z
  pinMode(EN_PIN_MZ, OUTPUT);
  pinMode(STEP_PIN_MZ, OUTPUT);
  pinMode(DIR_PIN_MZ, OUTPUT);
  digitalWrite(EN_PIN_MZ, LOW);

  MOT_A.setMaxSpeed(8000);
  MOT_B.setMaxSpeed(8000);
  MOT_Z.setMaxSpeed(4000);

  MOT_A.setAcceleration(2000);
  MOT_B.setAcceleration(2000);
  MOT_Z.setAcceleration(1000);

  MOT_A.setCurrentPosition(0);
  MOT_B.setCurrentPosition(0);
  MOT_Z.setCurrentPosition(0);

  pinMode(BTN_PIN_UP, INPUT_PULLUP);
  pinMode(BTN_PIN_DOWN, INPUT_PULLUP);
  pinMode(BTN_PIN_LEFT, INPUT_PULLUP);
  pinMode(BTN_PIN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_PIN_OK, INPUT_PULLUP);
  pinMode(BTN_Interup, INPUT_PULLUP);
}


void loop() {
  int deltaA = 0;
  int deltaB = 0;
  bouton_direction_tester = 0;

  //bouton_direction_tester = BTN_UP;
  if(digitalRead(BTN_PIN_UP) == LOW) {
    deltaA += speed;
    deltaB += speed;
  }

  //bouton_direction_tester = BTN_DOWN;
  if(digitalRead(BTN_PIN_DOWN) == LOW) {
    deltaA -= speed;
    deltaB -= speed;
  }

  //bouton_direction_tester = BTN_LEFT;
  if(digitalRead(BTN_PIN_LEFT) == LOW) {
    deltaA += speed;
    deltaB -= speed;
  }

  //bouton_direction_tester = BTN_RIGHT;
  if(digitalRead(BTN_PIN_RIGHT) == LOW) {
    deltaA -= speed;
    deltaB += speed;
  } 
  //Serial.print("delta A :"); Serial.print(deltaA); Serial.print(" delta B :"); Serial.println(deltaB);

  deltaX = (deltaA + deltaB) / 2;
  deltaY = (deltaA - deltaB) / 2;
  //Serial.print("delta X :"); Serial.print(deltaX); Serial.print(" delta Y :"); Serial.println(deltaY);

  // Update positions
  posX += deltaX;
  posY -= deltaY;
  //Serial.print("Position X :"); Serial.print(posX); Serial.print("  Position Y :"); Serial.println(posY);
  /*
  if(posX > MAX_POS_X) {
    //Permet juste déplacement selon axe des y théoriquement
    targetA = deltaY;
    targetB = -deltaY;
    Serial.println("Fin X");
  }
  else if (posX < 0) {
    targetA = deltaY;
    targetB = -deltaY;
    Serial.println("Début X");
  }
  // avoir techniquement un else if pour position zero limite siwchet ***
  else {
    targetA = deltaX + deltaY;
    targetB = deltaX - deltaY;
  }
    */

  targetA = deltaX + deltaY;
  targetB = deltaX - deltaY;

  

  // CoreXY mapping
  //targetA = deltaX + deltaY;
  //targetB = deltaX - deltaY;

  
  //Vérifier respecter les limites
  /*
  if(posX > MAX_POS_X) {
    Serial.print("A :"); Serial.print(targetA); Serial.print("  B :"); Serial.println(targetB);
    Serial.println("Fin X");
    Serial.print("delta A :"); Serial.print(deltaA); Serial.print(" delta B :"); Serial.println(deltaB);
    Serial.print("delta X :"); Serial.print(deltaX); Serial.print(" delta Y :"); Serial.println(deltaY);
    Serial.print("Position X :"); Serial.print(posX); Serial.print("  Position Y :"); Serial.println(posY);
    
    posX = MAX_POS_X;
    targetA = deltaY;
    targetB = -deltaY;
    MOT_A.stop();
    MOT_B.stop();
    Serial.print("A :"); Serial.print(targetA); Serial.print("  B :"); Serial.println(targetB);
  }
  else if(posX < 0) {
    Serial.print("A :"); Serial.print(targetA); Serial.print("  B :"); Serial.println(targetB);
    Serial.println("Début X");
    Serial.print("delta A :"); Serial.print(deltaA); Serial.print(" delta B :"); Serial.println(deltaB);
    Serial.print("delta X :"); Serial.print(deltaX); Serial.print(" delta Y :"); Serial.println(deltaY);
    Serial.print("Position X :"); Serial.print(posX); Serial.print("  Position Y :"); Serial.println(posY);

    posX = 0;
    targetA = deltaY;
    targetB = -deltaY;
    MOT_A.stop();
    MOT_B.stop();
    Serial.print("A :"); Serial.print(targetA); Serial.print("  B :"); Serial.println(targetB);
  }
  else {
    targetA = deltaX + deltaY;
    targetB = deltaX - deltaY;
  }

  if(posY > MAX_POS_Y) {
    Serial.println("Fin Y");
    deltaY -= speed;
  }
  //if(posY + deltaY < 0) deltaY = -posY;
  //Serial.print("delta X :"); Serial.print(deltaX); Serial.print(" delta Y :"); Serial.println(deltaY);
  */

  //Serial.print("A :"); Serial.print(targetA); Serial.print("  B :"); Serial.println(targetB);

  if ((targetA == 0) && (targetB == 0)) {
    MOT_A.stop(); // essayer avoir meullieur arrêt
    MOT_B.stop();
    Serial.println("Fin");

  } else {
    MOT_A.setSpeed(targetA);
    MOT_B.setSpeed(targetB);
  
    MOT_A.runSpeed();
    MOT_B.runSpeed();
  }

  /*
  while(toujoursMvtA || toujoursMvtB) {
    toujoursMvtA = MOT_A.run();
    toujoursMvtB = MOT_B.run();
  }
  */
  //Serial.println("Fin roulement");

  //MOT_A.run();
  //MOT_B.run();

  /*
  // Code pour détermine rlimite (test 3.2)
  if(((posX <= MAX_POS_X + 100) & (posX >= 0)) & ((posY >= 0) & (posY <= MAX_POS_Y + 100))) {
    MOT_A.run();
    MOT_B.run();
  }
  else {
    Serial.println("Fin");
    delay(10000);
  }
  */

  /*
  // Test déplacement chariot X diagonal 
  // (2026-03-25 11h) Cloé -> fonctionnelle 
  targetA -= speed;
  targetB += speed;
    
  MOT_A.moveTo(targetA);
  MOT_B.moveTo(targetB);

  MOT_A.run();
  MOT_B.run();
  */
}
