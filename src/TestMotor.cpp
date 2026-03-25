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
#define PIN_BTN_INTERRUPT 3 //D3
volatile TickType_t lastBtnInterrupt = 0;
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
int speed = 2; // il faut diminuer vitesse pour que ca fonctinne



long MAX_POS_X = 50000; // Adjust to your system's max travel in steps
long MAX_POS_Y = 50000;

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

#define BTN_OK 0
#define BTN_UP 1
#define BTN_DOWN 2
#define BTN_LEFT 3 // dia go
#define BTN_RIGHT 4 // roule dans le voe

long targetA = 0;
long targetB = 0;

//Mutex sur posX et posY ?
long posX = 0;
long posY = 0;

int bouton = BTN_UP;

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
}

void loop() {
  int deltaX = 0;
  int deltaY = 0;

  // Calcul des deltas selon boutons 
  if(bouton == BTN_UP)    deltaX += speed;
  if(bouton == BTN_DOWN)  deltaX -= speed;
  if(bouton == BTN_LEFT)  deltaY += speed;
  if(bouton == BTN_RIGHT) deltaY -= speed;
  
  // Respect des software maximum/min limits
  if(posX + deltaX > MAX_POS_X) deltaX = MAX_POS_X - posX;
  if(posY + deltaY > MAX_POS_Y) deltaY = MAX_POS_Y - posY;
  if(posX + deltaX < 0) deltaX = -posX;
  if(posY + deltaY < 0) deltaY = -posY;

  // Update positions
  posX += deltaX;
  posY += deltaY;

  // CoreXY mapping
  long targetA = posX + posY;
  long targetB = posX - posY;
    
  MOT_A.moveTo(targetA);
  MOT_B.moveTo(targetB);

  MOT_A.run();
  MOT_B.run();

  /*
  // Test déplacement moteur diagonal 2026-03-25 -> fonctionnelle 
  targetA -= speed;
  targetB -= speed;
    
  MOT_A.moveTo(targetA);
  MOT_B.moveTo(targetB);

  MOT_A.run();
  MOT_B.run();
  */
}
