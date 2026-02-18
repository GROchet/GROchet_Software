#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "event_groups.h"

//BOUTONS
#define BTN_PIN_UP 22
#define BTN_PIN_DOWN 25
#define BTN_PIN_LEFT 23
#define BTN_PIN_RIGHT 24
#define BTN_PIN_OK 26
#define PIN_BTN_INTERRUPT 3 //D3

//LIMIT SWITCHES
#define PIN_LMSW_INTER 2 //D2
#define LMTSW_Z 36
#define LMTSW_Y 38
#define LMTSW_X 40

//DYNAMIXEL
#define DXL_SERIAL   Serial1
#define DXL_DIR_PIN  -1
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
uint8_t id = 102;
uint8_t strength = 5; //2.69 mA per unit.
#define OPEN_POS 1000 //(A CHANGER)
#define CLOSED_POS 5000 //(A CHANGER)

enum SystemState {
  IDLE,
  LOWERING, //On descend l'axe Z
  CLOSING, //Pince fermée
  HOLDING_OBJECT, //Pince fermée, torque, mais pas de mouvement, le toutou est attrapé
  DROPPED, //Pince fermée, mais on a drop le toutou/ pas eu le toutou
  LIFTING, //Remonter Axe Z
  MOVING_TO_DROPZONE, //Se déplacer vers la zone de dépôt
  DROPPING, //Pince ouverte
  RETURN //Retour à la position de départ
};

enum ToutouState {
  ATTENTE,
  ATTRAPE,
  DROPPED
}
//MUTEX A METTRE DANS LE FUTUR
volatile SystemState currentState = IDLE;
volatile bool toutouAttrape = ATTENTE;
bool etatControlMoteur; //false = position, true = torque

void fermerPince ();
void ouvrirPince ();
void TaskdetecterToutou (void *pvParameters);
void TaskMotorControl (void *pvParameters);
void TaskStateControl (void *pvParameters);
void NotifySwitch();
void NotifyBtn();

EventGroupHandle_t inputEventGroup;
#define EVT_BTN_OK (1 << 0)
#define EVT_BTN_UP (1 << 1)
#define EVT_BTN_DOWN (1 << 2)
#define EVT_BTN_LEFT (1 << 3)
#define EVT_BTN_RIGHT (1 << 4)

void setup() {
  Serial.begin(115200);
  delay(2000);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  inputEventGroup = xEventGroupCreate();

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
  pinMode(LMTSW_Z, INPUT_PULLUP);
  pinMode(LMTSW_Y, INPUT_PULLUP);
  pinMode(LMTSW_X, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_LMSW_INTER), NotifySwitch, FALLING); //CHANGE OU FALLING ?

  //RTOS
  xTaskCreate(TaskdetecterToutou, "ToutouTask", 256, NULL, 1, NULL);
  xTaskCreate(TaskMotorControl, "MotorTask", 256, NULL, 1, NULL);
  xTaskCreate(TaskStateControl, "StateTask", 256, NULL, 1, NULL);

}

void loop() {
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
  for(;;){
    //Périodique, mais seulement quand on est en état de pince fermée.

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100)); //A Valider

    if (toutouAttrape == DROPPED){
      //Mettre un wait ?
      continue;
    }
    //On a le toutou, ou on est en attente.

    int32_t present_position = dxl.getPresentPosition(id);
    int32_t present_velocity = dxl.getPresentVelocity(id);
    //Serial.println("Position: " + String(present_position) + " Velocity: " + String(present_velocity));

    if (present_position > CLOSED_POS - 5) {
      dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, 0);
      toutouAttrape = DROPPED;
    }
    else if (abs(present_velocity) < 5 && present_position < CLOSED_POS - 10) {
      toutouAttrape = ATTRAPE; //Mettre un bool pour savoir si on a un objet ou s'il est dropped a la palce d'utiliser currentState ?
    }
  }
}

void TaskMotorControl (void *pvParameters) {
  (void) pvParameters;
  EventBits_t bits;

  for(;;){
    xEventGroupWaitBits(inputEventGroup, EVT_BTN_UP | EVT_BTN_DOWN | EVT_BTN_LEFT | EVT_BTN_RIGHT, pdFALSE, pdFALSE, portMAX_DELAY);
    //SET MOTEURS SELON LES BOUTONS APPUYES
    bits = xEventGroupGetBits(inputEventGroup);
  }
}

void TaskStateControl (void *pvParameters) {
  (void) pvParameters;
  EventBits_t bits;
  //Ca serait bien qu'aucune autre tache ne modifie CurrentState, comme ca on a pas besoin de le protéger.

  for(;;) {

    switch(currentState) {

      case IDLE:
        //Pendant ce temps, on accepte les déplacements haut-bas, gauche-droite
        // Wait for OK button, we CAN afford to wait infinitely here
        break;

      case LOWERING:
        //Séquence de mouvement vers le bas
        vTaskDelay(pdMS_TO_TICKS(2000)); //A remplacer par une condition de fin de mouvement
        currentState = CLOSING;
        break;

      case CLOSING:
        vTaskDelay(pdMS_TO_TICKS(500)); //A remplacer par une condition de fin de mouvement
        currentState = LIFTING;
        break;

      case LIFTING:
        //WAIT FOR X TO BE LIFTED, THEN MOVE TO DROPZONE
        vTaskDelay(pdMS_TO_TICKS(2000)); //A remplacer par une condition de détection de la levée du toutou.
        currentState = MOVING_TO_DROPZONE;
        break;

      case MOVING_TO_DROPZONE:
        //Séquence de mouvement vers la dropzone
        vTaskDelay(pdMS_TO_TICKS(2000)); //A remplacer par une condition de fin de mouvement
        currentState = DROPPING;
        break;

      case DROPPING:
        ouvrirPince();
        currentState = IDLE;
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void NotifySwitch(){
  //PENDANT SETUP, ON VEUT QUE LES (2 ou 3) LIMITSWITCHES SERVENT A TROUVER LE ZERO
  //En Situation normale, on veut que les limit switch servent à detecter les obstacles et à arrêter le moteur pour pas que ça arrache tout
}

void NotifyBtn(){
  //METTRE UNE QUEUE ?
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  bool up_state = digitalRead(BTN_PIN_UP);
  bool down_state = digitalRead(BTN_PIN_DOWN);
  bool left_state = digitalRead(BTN_PIN_LEFT);
  bool right_state = digitalRead(BTN_PIN_RIGHT);
  bool ok_state = digitalRead(BTN_PIN_OK);

  bool priority_btn_pressed = false;

  bool list[5] = {ok_state == LOW, up_state == LOW, down_state == LOW, left_state == LOW, right_state == LOW}; //Low because of Input_pullup

  EventBits_t evtList[5] = {EVT_BTN_OK,EVT_BTN_UP,EVT_BTN_DOWN,EVT_BTN_LEFT,EVT_BTN_RIGHT};

  //Si 2 boutons appuyés en meme temps, on garde que le plus prioritaire (ok > up > down > left > right)
  for (int i = 0; i < 5; i++) {
    if (list[i] && !priority_btn_pressed) {
      xEventGroupSetBitsFromISR(inputEventGroup, evtList[i], &xHigherPriorityTaskWoken);
      priority_btn_pressed = true;
    }
    else {
      xEventGroupClearBitsFromISR(inputEventGroup, evtList[i]);
    }
  }

  if(xHigherPriorityTaskWoken == pdTRUE){
    portYIELD_FROM_ISR();
  }

}

//Fonctions pour regler la force de la pince ?
//Fonctions pour communiquer avec le arduino
//Code limitSwitch

//AVOIR UN STATE EN_MOUVEMENT, pis quand on 