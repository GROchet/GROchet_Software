#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "event_groups.h"
#include <AccelStepper.h>

//BOUTONS
#define BTN_PIN_UP 22
#define BTN_PIN_DOWN 25
#define BTN_PIN_LEFT 23
#define BTN_PIN_RIGHT 24
#define BTN_PIN_OK 26
#define PIN_BTN_INTERRUPT 3 //D3

//LIMIT SWITCHES
#define PIN_LMSW_INTER 2 //D2
#define LMTSW_Y 38
#define LMTSW_X 40

//DYNAMIXEL
#define DXL_SERIAL   Serial1
#define DXL_DIR_PIN  -1
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
uint8_t id = 102;
uint8_t strength = 5; //2.69 mA per unit.
int speed = 50;
#define OPEN_POS 1000 //(A CHANGER)
#define CLOSED_POS 5000 //(A CHANGER)

//MOTEURS
#define FULLSTEP 4 // 4 fils par moteurs
#define STEPS_REV 4096
long posX = 0;
long posY = 0;
long MAX_POS_X = 50000; // Adjust to your system's max travel in steps
long MAX_POS_Y = 50000;

//Moteur 1
#define EN_PIN_M1           38 // Enable
#define DIR_PIN_M1          22 // Direction
#define STEP_PIN_M1         28 // Step
#define SW_RX_M1            48 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX_M1            50 // TMC2208/TMC2224 SoftwareSerial transmit pin

//Moteur 2
#define EN_PIN_M2           39 // Enable
#define DIR_PIN_M2          23 // Direction
#define STEP_PIN_M2         29 // Step
#define SW_RX_M2            49 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX_M2            51 // TMC2208/TMC2224 SoftwareSerial transmit pin

//Moteur Z
#define EN_PIN_MZ           43 // Enable
#define DIR_PIN_MZ          45 // Direction
#define STEP_PIN_MZ         42  // Step
#define SW_RX_MZ            47 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX_MZ            44 // TMC2208/TMC2224 SoftwareSerial transmit pin

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
bool etatControlMoteur; //false = position, true = torque

void fermerPince ();
void ouvrirPince ();
void TaskdetecterToutou (void *pvParameters);
void TaskMotorControl (void *pvParameters);
void TaskStateControl (void *pvParameters);
void NotifySwitch();
void NotifyBtn();
void homeXY();

EventGroupHandle_t inputEventGroup;
EventGroupHandle_t toutouEventGroup;
#define EVT_BTN_OK (1 << 0)
#define EVT_BTN_UP (1 << 1)
#define EVT_BTN_DOWN (1 << 2)
#define EVT_BTN_LEFT (1 << 3)
#define EVT_BTN_RIGHT (1 << 4)

#define EVT_TOUTOU_ATTRAPE (1 << 0)
#define EVT_TOUTOU_DROPPED (1 << 1)

void setup() {
  Serial.begin(115200);
  delay(2000);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  inputEventGroup = xEventGroupCreate();
  toutouEventGroup = xEventGroupCreate();

  //For finding id of your DYNAMIXEL, use this code.
  /*for (uint8_t i = 0; i < 253; i++) {
  if (dxl.ping(i)) {
    Serial.print("Found Dynamixel ID: ");
    Serial.println(i);
    id = i;
  }
  }*/

  //DYNAMIXEL 
  if (!dxl.ping(id)) {
    Serial.println("No Dynamixel found. Reality is disappointing.");
    while (1);
  }
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_CURRENT);
  dxl.torqueOn(id);
  etatControlMoteur = true; //Torque

  //BOUTONS
  pinMode(BTN_PIN_UP, INPUT_PULLUP); //Verifier avec Marcob input vs input_pullup (resistance interne ou pas)
  pinMode(BTN_PIN_DOWN, INPUT_PULLUP);
  pinMode(BTN_PIN_LEFT, INPUT_PULLUP);
  pinMode(BTN_PIN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_PIN_OK, INPUT_PULLUP);
  pinMode(PIN_BTN_INTERRUPT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_INTERRUPT), NotifyBtn, FALLING);

  //LIMIT SWITCHES
  pinMode(PIN_LMSW_INTER, INPUT_PULLUP);
  pinMode(LMTSW_Y, INPUT_PULLUP);
  pinMode(LMTSW_X, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_LMSW_INTER), NotifySwitch, FALLING); //CHANGE OU FALLING ?

  //MOTEURS
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

  MOT_A.setMaxSpeed(50000);
  MOT_B.setMaxSpeed(50000);
  MOT_Z.setMaxSpeed(50000);

  MOT_A.setAcceleration(10000);
  MOT_B.setAcceleration(10000);
  MOT_Z.setAcceleration(10000);

  MOT_A.setCurrentPosition(0);
  MOT_B.setCurrentPosition(0);
  MOT_Z.setCurrentPosition(0);

  //RTOS
  xTaskCreate(TaskdetecterToutou, "ToutouTask", 256, NULL, 1, NULL);
  xTaskCreate(TaskMotorControl, "MotorTask", 256, NULL, 1, NULL);
  xTaskCreate(TaskStateControl, "StateTask", 256, NULL, 1, NULL);

}

void loop() {
}

void homeXY() {
  Serial.println("Homing XY...");

  // --- Home X axis ---
  MOT_A.setSpeed(-1000);  // Both motors same direction
  MOT_B.setSpeed(-1000);

  while(digitalRead(LMTSW_X) == HIGH) {
    MOT_A.runSpeed();
    MOT_B.runSpeed();
  }
  // Stop and set positions
  MOT_A.setCurrentPosition(0);
  MOT_B.setCurrentPosition(0);
  posX = 0;

  // --- Home Y axis ---
  // For CoreXY, moving along Y: motors move in opposite directions
  MOT_A.setSpeed(-1000); // Adjust speed/direction according to your wiring
  MOT_B.setSpeed(1000);  // Opposite sign for Y

  while(digitalRead(LMTSW_Y) == HIGH) {
    MOT_A.runSpeed();
    MOT_B.runSpeed();
  }
  // Stop and reset positions
  MOT_A.setCurrentPosition(0);
  MOT_B.setCurrentPosition(0);
  posY = 0;

  Serial.println("Homing XY done.");
}

void fermerPince () { 
  if (!etatControlMoteur){
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_CURRENT);
    dxl.torqueOn(id);
    etatControlMoteur = true;
  }

  //Serrer toutou
  dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, strength);
}

void ouvrirPince () {
  if (etatControlMoteur){
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_POSITION);
    dxl.torqueOn(id);
    etatControlMoteur = false;
  }

  //Aller en position ouverte
  dxl.setGoalPosition(id, OPEN_POS); //Changer OPEN_POS
}

void TaskdetecterToutou (void *pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  EventBits_t bits;
  for(;;){
    //Périodique, mais seulement quand on est en état de pince fermée.

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100)); //A Valider
    
    bits = xEventGroupGetBits(toutouEventGroup);

    if (bits & EVT_TOUTOU_DROPPED){
      //Mettre un wait  ou un yield ?
      continue;
    }
    //On a le toutou, ou on est en attente.

    int32_t present_position = dxl.getPresentPosition(id);
    int32_t present_velocity = dxl.getPresentVelocity(id);
    //Serial.println("Position: " + String(present_position) + " Velocity: " + String(present_velocity));

    if (present_position > CLOSED_POS - 5) {
      dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, 0);
      xEventGroupSetBits(toutouEventGroup, EVT_TOUTOU_DROPPED); //Clear dans la tache de state control quand on a drop le toutou.
    }
    else if (abs(present_velocity) < 5 && present_position < CLOSED_POS - 10) {
      xEventGroupSetBits(toutouEventGroup, EVT_TOUTOU_ATTRAPE);
    }
  }
}


void TaskMotorControl (void *pvParameters) {
  (void) pvParameters;
  EventBits_t bits;

  for(;;){

    bits = xEventGroupGetBits(inputEventGroup);
    if ((bits & EVT_BTN_LEFT) | (bits & EVT_BTN_RIGHT) | (bits & EVT_BTN_UP) | (bits & EVT_BTN_DOWN)){
      int deltaX = 0;
      int deltaY = 0;

      // Calcul des deltas selon boutons
      if(bits & EVT_BTN_UP)    deltaX += speed;
      if(bits & EVT_BTN_DOWN)  deltaX -= speed;
      if(bits & EVT_BTN_LEFT)  deltaY += speed;
      if(bits & EVT_BTN_RIGHT) deltaY -= speed;

      // Respect des limit switches minimum
      if(digitalRead(LMTSW_X) == LOW && deltaX < 0) deltaX = 0;
      if(digitalRead(LMTSW_Y) == LOW && deltaY < 0) deltaY = 0;

      // Respect des software maximum limits
      if(posX + deltaX > MAX_POS_X) deltaX = MAX_POS_X - posX;
      if(posY + deltaY > MAX_POS_Y) deltaY = MAX_POS_Y - posY;

      // Update positions
      posX += deltaX;
      posY += deltaY;

      // CoreXY mapping
      long deltaMOTa = posX + posY;
      long deltaMOTb = posX - posY;

      MOT_A.moveTo(deltaMOTa);
      MOT_B.moveTo(deltaMOTb);

      // Execute non-blocking
      MOT_A.run();
      MOT_B.run();

      vTaskDelay(pdMS_TO_TICKS(10));
    }
    else {
      // Wait for any movement button press
      xEventGroupWaitBits(inputEventGroup, EVT_BTN_UP | EVT_BTN_DOWN | EVT_BTN_LEFT | EVT_BTN_RIGHT, pdFALSE, pdFALSE, portMAX_DELAY);
    }
  }
}

void TaskStateControl (void *pvParameters) {
  (void) pvParameters;
  EventBits_t bits;
  //aucune autre tache ne modifie CurrentState, comme ca on a pas besoin de le protéger.

  for(;;) {

    switch(currentState) {

      case IDLE://DONE
        //Pendant ce temps, on accepte les déplacements haut-bas, gauche-droite
        xEventGroupWaitBits(inputEventGroup, EVT_BTN_OK, pdTRUE, pdFALSE, portMAX_DELAY);
        currentState = LOWERING;
        break;

      case LOWERING:
        //Séquence de mouvement vers le bas
        vTaskDelay(pdMS_TO_TICKS(2000)); //A remplacer par une condition de fin de mouvement
        currentState = CLOSING;
        break;

      case CLOSING: //DONE
        xEventGroupClearBits(toutouEventGroup, EVT_TOUTOU_ATTRAPE | EVT_TOUTOU_DROPPED); //Au cas ou Ces bits auraient été set pendant la sequence de drop toutou
        fermerPince();
        xEventGroupWaitBits(toutouEventGroup, EVT_TOUTOU_ATTRAPE | EVT_TOUTOU_DROPPED, pdTRUE, pdFALSE, portMAX_DELAY);
        currentState = LIFTING;
        break;

      case LIFTING:
        //WAIT FOR X TO BE LIFTED, THEN MOVE TO DROPZONE
        vTaskDelay(pdMS_TO_TICKS(2000)); //A remplacer par une condition de détection de la levée du toutou.
        currentState = MOVING_TO_DROPZONE;
        break;

      case MOVING_TO_DROPZONE: //DONE
        //Séquence de mouvement vers la dropzone
        homeXY();
        ouvrirPince();
        currentState = DROPPING;
        break;

      case DROPPING: //DONE
        ouvrirPince();
        vTaskDelay(pdMS_TO_TICKS(1000));
        currentState = IDLE;
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void NotifySwitch(){
  //PENDANT SETUP, ON VEUT QUE LES (2 ou 3) LIMITSWITCHES SERVENT A TROUVER LE ZERO
  //En Situation normale, on veut que les limit switch servent à detecter les obstacles et à arrêter le moteur pour pas que ça arrache tout
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  bool switch_X = digitalRead(LMTSW_X) == LOW;
  bool switch_Y = digitalRead(LMTSW_Y) == LOW;

}

void NotifyBtn(){
  //METTRE UNE QUEUE ?
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  bool up_state = digitalRead(BTN_PIN_UP) == LOW; //Low because of Input_pullup
  bool down_state = digitalRead(BTN_PIN_DOWN) == LOW;
  bool left_state = digitalRead(BTN_PIN_LEFT) == LOW;
  bool right_state = digitalRead(BTN_PIN_RIGHT) == LOW;
  bool ok_state = digitalRead(BTN_PIN_OK) == LOW;

  bool list[5] = {ok_state, up_state, down_state, left_state, right_state}; //Low because of Input_pullup

  EventBits_t evtList[5] = {EVT_BTN_OK,EVT_BTN_UP,EVT_BTN_DOWN,EVT_BTN_LEFT,EVT_BTN_RIGHT};

  //Si 2 boutons appuyés en meme temps, on garde que le plus prioritaire (OK > AUTRES BOUTONS)
  if (ok_state) {
    xEventGroupSetBitsFromISR(inputEventGroup, EVT_BTN_OK, &xHigherPriorityTaskWoken);
  }
  else {
    xEventGroupClearBitsFromISR(inputEventGroup, EVT_BTN_OK);
    if (up_state) {
      xEventGroupSetBitsFromISR(inputEventGroup, EVT_BTN_UP, &xHigherPriorityTaskWoken);
    }
    else {
      xEventGroupClearBitsFromISR(inputEventGroup, EVT_BTN_UP);
    }
    if (down_state) {
      xEventGroupSetBitsFromISR(inputEventGroup, EVT_BTN_DOWN, &xHigherPriorityTaskWoken);
    }
    else {
      xEventGroupClearBitsFromISR(inputEventGroup, EVT_BTN_DOWN);
    }
    if (left_state) {
      xEventGroupSetBitsFromISR(inputEventGroup, EVT_BTN_LEFT, &xHigherPriorityTaskWoken);
    }
    else {
      xEventGroupClearBitsFromISR(inputEventGroup, EVT_BTN_LEFT);
    }
    if (right_state) {
      xEventGroupSetBitsFromISR(inputEventGroup, EVT_BTN_RIGHT, &xHigherPriorityTaskWoken);
    }
    else {
      xEventGroupClearBitsFromISR(inputEventGroup, EVT_BTN_RIGHT);
    }
  }

  if(xHigherPriorityTaskWoken == pdTRUE){
    portYIELD_FROM_ISR();
  }

}

//TODO

//Fonctions pour regler la force de la pince ?
//Code Axe Z
//Code de déplacement vers la dropzone
//Code déplacement XY
//FonctionS pour setter les zéros des moteurs stepper et dynamixel ?