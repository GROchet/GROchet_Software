#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "event_groups.h"
#include <AccelStepper.h>

//TO consider before running for the first Time

// strength of the grip
// OPEN_POS and CLOSED_POS of the Dynamixel

// speed of XY
// MAX_POS_X and MAX_POS_Y according to the system's dimensions (CORE_XY)
// MAX speed and acceleration of the stepper motors
// Serial Begin, for dynamixel and stepper_motors



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
int speed = 50;

//Mutex sur posX et posY ?
long posX = 0;
long posY = 0;

long MAX_POS_X = 50000; // Adjust to your system's max travel in steps
long MAX_POS_Y = 50000;

#define SerialGripper    Serial2 //A CHANGER SELON PORTS

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

long liftedZPos = 0; //Position de l'axe Z quand la pince est levée, à ajuster selon le système
long maxDownZPos = 10000; //Position de l'axe Z quand la pince est au plus bas, à ajuster selon le système

AccelStepper MOT_A = AccelStepper(AccelStepper::DRIVER, STEP_PIN_M1,DIR_PIN_M1); //Moteur gauche
AccelStepper MOT_B = AccelStepper(AccelStepper::DRIVER, STEP_PIN_M2,DIR_PIN_M2); //Moteur droite
AccelStepper MOT_Z = AccelStepper(AccelStepper::DRIVER, STEP_PIN_MZ,DIR_PIN_MZ); //Moteur Z

enum SystemState {
  DIFF_CHOOSE, //State pour choisir la difficulté (bouton OK pour valider la difficulté choisie, et passer à IDLE)
  IDLE,
  LOWERING, //On descend l'axe Z
  CLOSING, //Pince fermée
  LIFTING, //Remonter Axe Z
  MOVING_TO_DROPZONE, //Se déplacer vers la zone de dépôt
  DROPPING, //Pince ouverte
};
volatile SystemState currentState = IDLE;

bool toutouAttrape = false; // Variable globale pour stocker l'état du toutou attrapé

void TaskMotorControl (void *pvParameters);
void TaskStateControl (void *pvParameters);
void TaskDetectToutou(void *pvParameters);
void NotifySwitch();
void NotifyBtn();
void homeXY();
uint8_t crc8(uint8_t *data, uint8_t len);
void gripperSend(uint8_t msg);
uint8_t gripperReceive(uint32_t timeoutMs = 500);

EventGroupHandle_t inputEventGroup;
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t detectToutouTaskHandle = NULL;

SemaphoreHandle_t toutouMutex;
SemaphoreHandle_t gripperMutex; //For serial communication with OpenRB-150

#define EVT_BTN_OK (1 << 0)
#define EVT_BTN_UP (1 << 1)
#define EVT_BTN_DOWN (1 << 2)
#define EVT_BTN_LEFT (1 << 3)
#define EVT_BTN_RIGHT (1 << 4)

void setup() {
  Serial.begin(115200);
  SerialGripper.begin(9600);
  delay(2000);

  inputEventGroup = xEventGroupCreate();
  limitEventGroup = xEventGroupCreate();

  toutouMutex = xSemaphoreCreateMutex();
  gripperMutex = xSemaphoreCreateMutex();

  //BOUTONS
  //----------------
  pinMode(BTN_PIN_UP, INPUT_PULLUP); //Verifier avec Marcob input vs input_pullup (resistance interne ou pas)
  pinMode(BTN_PIN_DOWN, INPUT_PULLUP);
  pinMode(BTN_PIN_LEFT, INPUT_PULLUP);
  pinMode(BTN_PIN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_PIN_OK, INPUT_PULLUP);
  pinMode(PIN_BTN_INTERRUPT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_INTERRUPT), NotifyBtn, CHANGE);

  //LIMIT SWITCHES
  //----------------
  pinMode(PIN_LMSW_INTER, INPUT_PULLUP);
  pinMode(LMTSW_Y, INPUT_PULLUP);
  pinMode(LMTSW_X, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_LMSW_INTER), NotifySwitch, FALLING); //CHANGE OU FALLING ?

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

  //RTOS
  //----------------
  xTaskCreate(TaskMotorControl, "MotorTask", 256, NULL, 3, &motorTaskHandle);
  xTaskCreate(TaskStateControl, "StateTask", 512, NULL, 1, NULL);
  xTaskCreate(TaskDetectToutou, "DetectToutouTask", 256, NULL, 2, &detectToutouTaskHandle);
}

void loop() {
}

void homeXY() {
  Serial.println("Homing XY...");

  // Clear any stale limit bits first
  xEventGroupClearBits(limitEventGroup, EVT_LIMIT_X | EVT_LIMIT_Y);

  MOT_A.stop(); 
  MOT_A.setCurrentPosition(MOT_A.currentPosition());
  MOT_B.stop(); 
  MOT_B.setCurrentPosition(MOT_B.currentPosition());

  // --- Home X axis ---
  MOT_A.setSpeed(-1000);
  MOT_B.setSpeed(-1000);

  while (!(xEventGroupGetBits(limitEventGroup) & EVT_LIMIT_X)) {
    MOT_A.runSpeed();
    MOT_B.runSpeed();
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  MOT_A.setCurrentPosition(0);
  MOT_B.setCurrentPosition(0);
  posX = 0;

  // Clear bit so it doesn't interfere with Y
  xEventGroupClearBits(limitEventGroup, EVT_LIMIT_X);

  // --- Home Y axis ---
  MOT_A.setSpeed(-1000);
  MOT_B.setSpeed(1000);

  while (!(xEventGroupGetBits(limitEventGroup) & EVT_LIMIT_Y)) {
    MOT_A.runSpeed();
    MOT_B.runSpeed();
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  MOT_A.setCurrentPosition(0);
  MOT_B.setCurrentPosition(0);
  posY = 0;

  xEventGroupClearBits(limitEventGroup, EVT_LIMIT_Y);

  Serial.println("Homing XY done.");
}

void TaskMotorControl (void *pvParameters) {
  (void) pvParameters;
  EventBits_t bits;
  EventBits_t limitBits;

  for(;;){

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    for(;;){

      bits = xEventGroupGetBits(inputEventGroup);
      limitBits = xEventGroupGetBits(limitEventGroup);
      /*
      //Limit Switch Protection
      if ((limitBits & EVT_LIMIT_X) && posX <= 0){
        MOT_A.stop();
        MOT_B.stop();
        posX = 0;

        // Cancel any queued motion
        MOT_A.moveTo(MOT_A.currentPosition());
        MOT_B.moveTo(MOT_B.currentPosition());
      }

      if ((limitBits & EVT_LIMIT_Y) && posY <= 0){
        MOT_A.stop();
        MOT_B.stop();
        posY = 0;

        MOT_A.moveTo(MOT_A.currentPosition());
        MOT_B.moveTo(MOT_B.currentPosition());
      }*/

      //When a Button is pressed
      if (bits & (EVT_BTN_LEFT | EVT_BTN_RIGHT | EVT_BTN_UP | EVT_BTN_DOWN)){
        int deltaX = 0;
        int deltaY = 0;

        // Calcul des deltas selon boutons
        if(bits & EVT_BTN_UP)    deltaX += speed;
        if(bits & EVT_BTN_DOWN)  deltaX -= speed;
        if(bits & EVT_BTN_LEFT)  deltaY += speed;
        if(bits & EVT_BTN_RIGHT) deltaY -= speed;
      
        // Respect des limit switches minimum
        if(deltaX < 0 && (limitBits & EVT_LIMIT_X)) deltaX = 0;
        if(deltaY < 0 && (limitBits & EVT_LIMIT_Y)) deltaY = 0;

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
      }

      // always update steppers
      MOT_A.run();
      MOT_B.run();

      // Check if we received a new notification telling us to STOP
      if (ulTaskNotifyTake(pdTRUE, 0)) {
        break;  // Exit manual mode immediately
      }
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
}

void TaskStateControl (void *pvParameters) {
  (void) pvParameters;
  //aucune autre tache ne modifie CurrentState, comme ca on a pas besoin de le protéger.

  for(;;) {

    switch(currentState) {
      case DIFF_CHOOSE:
        xTaskNotifyGive(motorTaskHandle);   // Enable manual control
        xEventGroupWaitBits(inputEventGroup, EVT_BTN_OK, pdTRUE, pdFALSE, portMAX_DELAY);
        xTaskNotifyGive(motorTaskHandle);   // Send stop signal
        currentState = IDLE;
      case IDLE://DONE
        Serial.println("IDLE: Use buttons to move, OK to calibrate.");
        xTaskNotifyGive(motorTaskHandle);   // Enable manual control
        xEventGroupWaitBits(inputEventGroup, EVT_BTN_OK, pdTRUE, pdFALSE, portMAX_DELAY);
        xTaskNotifyGive(motorTaskHandle);   // Send stop signal
        Serial.println("Transition to LOWERING");
        currentState = LOWERING;
        break;

      case LOWERING:
        //Séquence de mouvement vers le bas
        MOT_Z.setSpeed(speed);
        while (1) {
          if (xEventGroupGetBits(inputEventGroup) & EVT_BTN_OK) break;
          else if (MOT_Z.currentPosition() >= maxDownZPos) break; // On s'assure de pas descendre plus que la position levée, au cas où le limit switch ne marche pas
          MOT_Z.runSpeed(); // actually step the motor
          vTaskDelay(pdMS_TO_TICKS(1));
        }
        xEventGroupClearBits(inputEventGroup, EVT_BTN_OK); // clear the bit
        MOT_Z.stop();

        Serial.println("Transition to CLOSING");
        currentState = CLOSING;
        break;

      case CLOSING:{  
        xSemaphoreTake(gripperMutex, portMAX_DELAY);
        gripperSend(0x02);
        xSemaphoreGive(gripperMutex);      
        //Attendre que le toutou soit attrapé ou pas (message de retour de OpenRB-150)
        uint8_t resp1 = 0x04; //Start as "moving"
        while((resp1 &  0x01) == 0 && (resp1 & 0x02) == 0){ //tant que ni toutou attrapé ni rien attrapé
          xSemaphoreTake(gripperMutex, portMAX_DELAY);

          gripperSend(0x04);
          resp1 = gripperReceive(); // Wait for response with timeout

          xSemaphoreGive(gripperMutex);
          vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (resp1 & 0x01) {
          Serial.println("Toutou attrapé !");
          xSemaphoreTake(toutouMutex, portMAX_DELAY);
          toutouAttrape = true;
          xSemaphoreGive(toutouMutex);
          //On passe a lifting. MAIS ON DOIT CONTINUER DE DETECTER TOUTOU EN MEME TEMPS, JUSQUA DROPPING... créer une nouvelle tache ?
          xTaskNotifyGive(detectToutouTaskHandle); // Lancer la tache de detection du toutou attrapé ou pas
        } 
        else if (resp1 & 0x02) {
          Serial.println("Rien attrapé.");
          xSemaphoreTake(toutouMutex, portMAX_DELAY);
          toutouAttrape = false;
          xSemaphoreGive(toutouMutex);
          //Variable pour PAS call ouvrirPince tantot.
        }
        currentState = LIFTING;
        break;
      }
      case LIFTING:
        //WAIT FOR Z TO BE LIFTED, THEN MOVE TO DROPZONE
        MOT_Z.setSpeed(-speed);
        while(MOT_Z.currentPosition() > liftedZPos){
          MOT_Z.runSpeed(); // actually step the motor
          vTaskDelay(pdMS_TO_TICKS(5));
        }
        MOT_Z.stop();
        currentState = MOVING_TO_DROPZONE;
        break;

      case MOVING_TO_DROPZONE: //DONE
        //Séquence de mouvement vers la dropzone
        homeXY();
        currentState = DROPPING;
        break;

      case DROPPING:{ //DONE;

        xSemaphoreTake(toutouMutex, portMAX_DELAY);
        bool local = toutouAttrape;
        xSemaphoreGive(toutouMutex);

        if (local) {
          xSemaphoreTake(gripperMutex, portMAX_DELAY);
          gripperSend(0x01); // ouvrir pince
          xSemaphoreGive(gripperMutex);

          vTaskDelay(pdMS_TO_TICKS(1000));
          xTaskNotifyGive(detectToutouTaskHandle); // Stop la tache de detection du toutou, et ouvrir la pince si elle est encore fermée
        }
        xSemaphoreTake(toutouMutex, portMAX_DELAY);
        toutouAttrape = false;
        xSemaphoreGive(toutouMutex);
        currentState = IDLE;
        break;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void NotifySwitch(){
  //PENDANT SETUP, ON VEUT QUE LES (2 ou 3) LIMITSWITCHES SERVENT A TROUVER LE ZERO
  //En Situation normale, on veut que les limit switch servent à detecter les obstacles et à arrêter le moteur pour pas que ça arrache tout
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (digitalRead(LMTSW_X) == LOW) {
    xEventGroupSetBitsFromISR(limitEventGroup, EVT_LIMIT_X, &xHigherPriorityTaskWoken);
  }
  if (digitalRead(LMTSW_Y) == LOW) {
    xEventGroupSetBitsFromISR(limitEventGroup, EVT_LIMIT_Y, &xHigherPriorityTaskWoken);
  }
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }

}

void NotifyBtn(){
  //METTRE UNE QUEUE ?
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  //Ignore button bouncing
  uint32_t now = xTaskGetTickCountFromISR();
  if (now - lastBtnInterrupt < pdMS_TO_TICKS(DEBOUNCE_MS)) {
      return;
  }
  lastBtnInterrupt = now;

  bool up_state = digitalRead(BTN_PIN_UP) == LOW;
  bool down_state = digitalRead(BTN_PIN_DOWN) == LOW;
  bool left_state = digitalRead(BTN_PIN_LEFT) == LOW;
  bool right_state = digitalRead(BTN_PIN_RIGHT) == LOW;
  bool ok_state = digitalRead(BTN_PIN_OK) == LOW;

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

void TaskDetectToutou (void *pvParameters) {
  (void) pvParameters;
  for(;;){

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    for(;;){
      //Lire les capteurs de la pince pour detecter si le toutou est attrapé ou pas<
      xSemaphoreTake(gripperMutex, portMAX_DELAY);
      gripperSend(0x04); 
      uint8_t resp1 = 0x04; //Start as "moving"
      resp1 = gripperReceive(); // Wait for response with timeout
      xSemaphoreGive(gripperMutex);

      vTaskDelay(pdMS_TO_TICKS(100));

      //Mettre à jour la variable toutouAttrape en conséquence
      if (resp1 & 0x02) { //Toutou laché
        xSemaphoreTake(toutouMutex, portMAX_DELAY);
        toutouAttrape = false;
        xSemaphoreGive(toutouMutex);
        //Variable pour PAS call ouvrirPince tantot.
      }

      xSemaphoreTake(toutouMutex, portMAX_DELAY);
      bool local = toutouAttrape;
      xSemaphoreGive(toutouMutex);

      // Check if we received a new notification telling us to STOP 
      if (ulTaskNotifyTake(pdTRUE, 0) || local == false) { 
        break;  // Exit detection loop immediately
      }
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}

uint8_t crc8 (uint8_t *data, uint8_t len){
  uint8_t crc = 0x00; 
  for (uint8_t i = 0;i<len;i++){
    crc ^= data[i];
    for (uint8_t b = 0;b<8;b++){
      if(crc & 0x80) crc = (crc << 1) ^ 0x07;
      else crc <<= 1;
    }
  }
  return crc;
}

void gripperSend(uint8_t msg) {
  uint8_t frame[2] = { msg, crc8(&msg, 1) };
  SerialGripper.write(frame, 2);
}

uint8_t gripperReceive(uint32_t timeoutMs = 500) {
  uint32_t start = millis();
  while (SerialGripper.available() < 2) {
    if (millis() - start > timeoutMs) return 0xFF; // timeout
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  uint8_t resp = SerialGripper.read();
  uint8_t crc  = SerialGripper.read();
  if (crc != crc8(&resp, 1)) return 0xFF; // CRC mismatch
  return resp;
}

//TODO
//Fonctions pour regler la force de la pince ?
//Code Axe Z
//FonctionS pour setter les zéros de l'axe Z et Pince ? lors de l'init
//Set speed et non position pour stepper motor.