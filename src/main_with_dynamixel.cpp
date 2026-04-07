#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "event_groups.h"
#include <AccelStepper.h>
#include <ArduinoJson.h>
#include <Dynamixel2Arduino.h>

//À considérer avant de Run la premiere fois :

// Force de la pince
// OPEN_POS et CLOSED_POS du dynamixel (pince)

// vitesse du CoreXY
// MAX_POS_X and MAX_POS_Y according to the system's dimensions (CORE_XY)
// MAX speed and acceleration of the stepper motors
// Serial Begin, for dynamixel and stepper_motors

//-------------------
//Difficulté
//-------------------
int16_t temps[] = {50,100,150}; //A ajuster selon les tests, en s, pour chaque difficulté (easy, medium, hard). Temps pendant lequel le toutou doit être attrapé pour valider la prise et passer à l'étape suivante. Peut être différent selon la difficulté choisie.
int16_t force[] = {50,100,150}; //A ajuster selon les tests, en unités brutes du dynamixel
int16_t speed[] = {1000,1250,1500}; //A ajuster selon les tests, en unités de vitesse du stepper (peut être différent selon le système et les moteurs utilisés)
int8_t difficulty = 0; // 0 = easy, 1 = medium, 2 = hard. A ajuster selon les tests

String ledColor = "rose"; //A ajuster selon les tests, couleur de la LED pour chaque difficulté (easy, medium, hard)

// -------------------
//BOUTONS
// -------------------
#define BTN_PIN_UP 22
#define BTN_PIN_DOWN 25
#define BTN_PIN_LEFT 23
#define BTN_PIN_RIGHT 24
#define BTN_PIN_OK 26
#define PIN_BTN_INTERRUPT 3 //D3
volatile TickType_t lastBtnInterrupt = 0;
#define DEBOUNCE_MS 20

#define EVT_BTN_OK (1 << 0)
#define EVT_BTN_UP (1 << 1)
#define EVT_BTN_DOWN (1 << 2)
#define EVT_BTN_LEFT (1 << 3)
#define EVT_BTN_RIGHT (1 << 4)

// -------------------
//LIMIT SWITCHES
// -------------------
#define PIN_LMSW_INTER 2 //D2
#define LMTSW_Y 38
#define LMTSW_X 40
EventGroupHandle_t limitEventGroup;
#define EVT_LIMIT_X (1 << 0)
#define EVT_LIMIT_Y (1 << 1)

// -------------------
//PINCE DYNAMIXEL
// -------------------
#define DXL_SERIAL      Serial1
#define DXL_DIR_PIN     -1

int16_t OPEN_POS = 3406;
int16_t CLOSED_POS = 6805;
int16_t ACTUAL_POS = 0;

#define MOVE_CURRENT    300 // Courant à appliquer pour déplacer la pince (unités brutes)
#define GRIP_CURRENT    50 // Courant à appliquer pour fermer la pince (unités brutes)
#define STALL_CURRENT   40 // Courant en dessous duquel on considère que le moteur est en stall (unités brutes)
#define VEL_THRESHOLD   5 // Vitesse en dessous de laquelle on considère que le moteur est à l'arrêt (unités brutes)
#define STALL_CONFIRM   300 // Temps de suite que le moteur doit être à l'arrêt pour confirmer un stall (ms)

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
uint8_t  id          = 102;

// -------------------
//MOTEURS
// -------------------

#define FULLSTEP 4 // 4 fils par moteurs
#define STEPS_REV 4096

float targetA = 0;
float targetB = 0;

int deltaX = 0;
int deltaY = 0;
 
long posX = 6550;
long posY = 0;

long MAX_POS_X = 13100;
long MAX_POS_Y = 12998;

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

// -------------------
// STATE MACHINE
// -------------------

enum SystemState {
  SETUP,
  DIFF_CHOOSE, //State pour choisir la difficulté (bouton OK pour valider la difficulté choisie, et passer à IDLE)
  IDLE,
  LOWERING, //On descend l'axe Z
  CLOSING, //Pince fermée
  LIFTING, //Remonter Axe Z
  MOVING_TO_DROPZONE, //Se déplacer vers la zone de dépôt
  DROPPING, //Pince ouverte
};

volatile SystemState currentState = IDLE;

//Taches et fonctions
void TaskMotorControl (void *pvParameters);
void TaskStateControl (void *pvParameters);
void TaskCommJsonReceive (void *pvParameters);
void TaskCommJsonSend(void *pvParameters);
void NotifySwitch();
void NotifyOkButton();
void homeXY();
String buildStatusJson();
void ouvrirPince();
void fermerPince();

//Global variables for RTOS synchronization
EventGroupHandle_t inputEventGroup;
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t detectToutouTaskHandle = NULL;

void setup() {
	Serial.begin(115200); // OU 115200 selon ce qui est choisi pour le debug
	delay(2000);

	//DYNAMIXEL
	// -------------------
	dxl.begin(57600);
	dxl.setPortProtocolVersion(2.0);

	for (uint8_t i = 1; i < 253; i++) {
        if (dxl.ping(i)) {
            id = i;
        }
    }
	dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_EXTENDED_POSITION);
    dxl.torqueOn(id);
    ACTUAL_POS = dxl.getPresentPosition(id); // Lire la position actuelle à l'initialisation
    dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, MOVE_CURRENT);

	//BOUTONS
	//----------------
	pinMode(BTN_PIN_UP, INPUT_PULLUP); //Verifier avec Marcob input vs input_pullup (resistance interne ou pas)
	pinMode(BTN_PIN_DOWN, INPUT_PULLUP);
	pinMode(BTN_PIN_LEFT, INPUT_PULLUP);
	pinMode(BTN_PIN_RIGHT, INPUT_PULLUP);
	pinMode(BTN_PIN_OK, INPUT_PULLUP);
	pinMode(PIN_BTN_INTERRUPT, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(PIN_BTN_INTERRUPT), NotifyOkButton, FALLING); // Only OK

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
	inputEventGroup = xEventGroupCreate();
	limitEventGroup = xEventGroupCreate();

	xTaskCreate(TaskMotorControl, "MotorTask", 256, NULL, 3, &motorTaskHandle);
	xTaskCreate(TaskStateControl, "StateTask", 512, NULL, 4, NULL);
	xTaskCreate(TaskCommJsonSend,    "CommSend", 2048, NULL, 1, NULL);
	xTaskCreate(TaskCommJsonReceive, "CommRecv", 512, NULL, 2, NULL);
}

void loop() {
}

void homeXY() {

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

}

void TaskMotorControl(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        int deltaA = 0;
        int deltaB = 0;

        // Read direction buttons directly (X/Y only)
        if (digitalRead(BTN_PIN_UP) == LOW)    { deltaA += speed[difficulty]; deltaB += speed[difficulty]; }
        if (digitalRead(BTN_PIN_DOWN) == LOW)  { deltaA -= speed[difficulty]; deltaB -= speed[difficulty]; }
        if (digitalRead(BTN_PIN_LEFT) == LOW)  { deltaA += speed[difficulty]; deltaB -= speed[difficulty]; }
        if (digitalRead(BTN_PIN_RIGHT) == LOW) { deltaA -= speed[difficulty]; deltaB += speed[difficulty]; }

        if (deltaA != 0 || deltaB != 0) {
            MOT_A.move(deltaA);
            MOT_B.move(deltaB);
        } else {
            MOT_A.stop();
            MOT_B.stop();
        }

        // Only run the X/Y motors
        MOT_A.run();
        MOT_B.run();

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void TaskStateControl (void *pvParameters) {
  (void) pvParameters;

  for(;;) {

	switch(currentState) {
		case SETUP:
			//calibrerPinceEtAxeZ();
			currentState = IDLE;
			break;
		case DIFF_CHOOSE:
		  	xTaskNotifyGive(motorTaskHandle);   // Enable manual control
			xEventGroupWaitBits(inputEventGroup, EVT_BTN_OK, pdTRUE, pdFALSE, portMAX_DELAY);
			xTaskNotifyGive(motorTaskHandle);   // Send stop signal
			currentState = IDLE;
			break;
	    case IDLE:
			xTaskNotifyGive(motorTaskHandle);   // Enable manual control
			xEventGroupWaitBits(inputEventGroup, EVT_BTN_OK, pdTRUE, pdFALSE, portMAX_DELAY);
			xTaskNotifyGive(motorTaskHandle);   // Send stop signal
			currentState = LOWERING;
			break;
	    case LOWERING:
			//Séquence de mouvement vers le bas
			MOT_Z.setSpeed(-speed[difficulty]);
			while(1) {
				if (xEventGroupGetBits(inputEventGroup) & EVT_BTN_OK) break;
				else if (MOT_Z.currentPosition() > maxDownZPos+50) break; // On s'assure de pas descendre plus que la position min, au cas où le limit switch ne marche pas
				MOT_Z.runSpeed(); // actually step the motor
				vTaskDelay(pdMS_TO_TICKS(2));
			}
			xEventGroupClearBits(inputEventGroup, EVT_BTN_OK); // clear the bit
			MOT_Z.stop();

			currentState = CLOSING;
			break;

	    case CLOSING:{  
			fermerPince();
			while (ACTUAL_POS < CLOSED_POS - 50) { // Tant que la pince n'est pas presque fermée
				vTaskDelay(pdMS_TO_TICKS(50));
				ACTUAL_POS = dxl.getPresentPosition(id);
			}		
			currentState = LIFTING;
			break;
	    }
	    case LIFTING:
			//WAIT FOR Z TO BE LIFTED, THEN MOVE TO DROPZONE
			MOT_Z.setSpeed(speed[difficulty]);
			while(MOT_Z.currentPosition() < liftedZPos-50){
				MOT_Z.runSpeed(); // actually step the motor
				vTaskDelay(pdMS_TO_TICKS(5));
			}
			MOT_Z.stop();
			currentState = MOVING_TO_DROPZONE;
			break;

	    case MOVING_TO_DROPZONE: //DONE
			//Séquence de mouvement vers la dropzone
			//homeXY();
			currentState = DROPPING;
			break;

	    case DROPPING:{ //DONE;
			ouvrirPince();
			while (ACTUAL_POS > OPEN_POS + 50) { // Tant que la pince n'est pas presque ouverte
				vTaskDelay(pdMS_TO_TICKS(50));
				ACTUAL_POS = dxl.getPresentPosition(id);
			}
			currentState = IDLE;
			break;
	    }
	}
	vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void TaskCommJsonSend(void *pvParameters) {
    (void) pvParameters;

    for(;;) {
        String jsonMsg = buildStatusJson();
        Serial.println(jsonMsg);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void NotifySwitch(){
	//PENDANT SETUP, ON VEUT QUE LES 2 LIMITSWITCHES SERVENT A TROUVER LE ZERO
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

void NotifyOkButton() {
    TickType_t now = xTaskGetTickCountFromISR();
    if (now - lastBtnInterrupt < pdMS_TO_TICKS(DEBOUNCE_MS)) return;
    lastBtnInterrupt = now;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(inputEventGroup, EVT_BTN_OK, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

void ouvrirPince() {
    dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, MOVE_CURRENT);
    dxl.setGoalPosition(id, OPEN_POS, UNIT_RAW);
}

void fermerPince() {
    dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, GRIP_CURRENT);
    dxl.setGoalPosition(id, CLOSED_POS, UNIT_RAW);
}

String buildStatusJson() {
    StaticJsonDocument<1024> doc;   // or JsonDocument doc(1024) for Static

    // Temps et niveaux
    doc["time_facile"] = temps[0];
    doc["time_medium"] = temps[1];
    doc["time_expert"] = temps[2];

    doc["force_facile"] = force[0];
    doc["force_medium"] = force[1];
    doc["force_expert"] = force[2];

    doc["speed_facile"] = speed[0];
    doc["speed_medium"] = speed[1];
    doc["speed_expert"] = speed[2];

    doc["led_color"] = ledColor;

	JsonObject buttons = doc["buttons"].to<JsonObject>();
	EventBits_t bits = xEventGroupGetBits(inputEventGroup);
	buttons["haut"] = (bits & EVT_BTN_UP) != 0;
	buttons["bas"] = (bits & EVT_BTN_DOWN) != 0;
	buttons["gauche"] = (bits & EVT_BTN_LEFT) != 0;
	buttons["droite"] = (bits & EVT_BTN_RIGHT) != 0;
	buttons["ok"] = (bits & EVT_BTN_OK) != 0;

    // System state
    doc["system_state"] = (int) currentState;

    // Pince
    JsonObject pince = doc["pince"].to<JsonObject>();
    pince["pos_ouverte"] = OPEN_POS;
    pince["pos_fermee"] = CLOSED_POS;
    pince["pos_actuelle"] = ACTUAL_POS;

    // XY
    JsonObject xy = doc["xy"].to<JsonObject>();
    xy["max_x"] = MAX_POS_X;
    xy["max_y"] = MAX_POS_Y;
    xy["pos_x"] = posX;
    xy["pos_y"] = posY;

    // Z axis
    JsonObject z_axis = doc["z_axis"].to<JsonObject>();
    z_axis["max_height"] = maxDownZPos;
    z_axis["min_height"] = liftedZPos;
    z_axis["current_height"] = MOT_Z.currentPosition();

    String output;
    serializeJson(doc, output);
    return output;
}

void TaskCommJsonReceive(void *pvParameters) {
    (void) pvParameters;

    for (;;) {

        // Check for incoming JSON
        if (Serial.available()) {
            String incoming = Serial.readStringUntil('\n');
            incoming.trim();

            StaticJsonDocument<512> doc;
            DeserializationError err = deserializeJson(doc, incoming);
            if (err) {
                vTaskDelay(pdMS_TO_TICKS(50));
                continue; // ignore malformed JSON
            }

            const char* type   = doc["type"];
            const char* action = doc["action"];
            const char* champ  = doc["champ"];

            // ── COMMANDES ────────────────────────────────────
            if (strcmp(type, "commande") == 0) {

                if (strcmp(action, "urgence") == 0) {
                    MOT_A.stop(); MOT_B.stop(); MOT_Z.stop();
                    currentState = IDLE;
                }
                else if (strcmp(action, "reinitialiser") == 0) {
                    currentState = SETUP;
                }
                else if (strcmp(action, "init") == 0) {
                    homeXY();
                }

                // Pince
                else if (strcmp(action, "ouvrir_pince") == 0)  ouvrirPince();
                else if (strcmp(action, "fermer_pince") == 0)  fermerPince();
                else if (strcmp(action, "moitie_pince") == 0) {
                    int32_t mid = (OPEN_POS + CLOSED_POS) / 2;
                    dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, MOVE_CURRENT);
                    dxl.setGoalPosition(id, mid, UNIT_RAW);
                }
                else if (strcmp(action, "ouvrir_manuel") == 0){
                    int32_t target = dxl.getPresentPosition(id) - 100;
                    dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, MOVE_CURRENT);
                    dxl.setGoalPosition(id, target, UNIT_RAW);
                }
                else if (strcmp(action, "fermer_manuel") == 0) {
                    int32_t target = dxl.getPresentPosition(id) + 100;
                    dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, MOVE_CURRENT);
                    dxl.setGoalPosition(id, target, UNIT_RAW);
                }

                // XY
                else if (strcmp(action, "deplacer_x_plus") == 0) {
                    posX += speed[difficulty];
                    MOT_A.moveTo(posX + posY); 
                    MOT_B.moveTo(posX - posY);
                }
                else if (strcmp(action, "deplacer_x_moins") == 0) {
                    posX -= speed[difficulty];
                    MOT_A.moveTo(posX + posY); 
                    MOT_B.moveTo(posX - posY);
                }
                else if (strcmp(action, "deplacer_y_plus") == 0) {
                    posY += speed[difficulty];
                    MOT_A.moveTo(posX + posY); 
                    MOT_B.moveTo(posX - posY);
                }
                else if (strcmp(action, "deplacer_y_moins") == 0) {
                    posY -= speed[difficulty];
                    MOT_A.moveTo(posX + posY); 
                    MOT_B.moveTo(posX - posY);
                }
                else if (strcmp(action, "position_initiale_xy") == 0) homeXY();
                else if (strcmp(action, "position_milieu_xy") == 0) {
                    posX = MAX_POS_X / 2; 
                    posY = MAX_POS_Y / 2;
                    MOT_A.moveTo(posX + posY); 
                    MOT_B.moveTo(posX - posY);
                }

                // Z
                else if (strcmp(action, "deplacer_z_haut") == 0) {
                    //MOT_Z.moveTo(MOT_Z.currentPosition() + 100);
                }
                else if (strcmp(action, "deplacer_z_bas") == 0) {
                    //MOT_Z.moveTo(MOT_Z.currentPosition() - 100);
                }
                else if (strcmp(action, "position_haut_z") == 0) {
                    //MOT_Z.moveTo(liftedZPos);
                }
                else if (strcmp(action, "position_bas_z") == 0) {
                    //MOT_Z.moveTo(maxDownZPos);
                }
            }

            // ── PERSONNALISATION ─────────────────────────────
            else if (strcmp(type, "personnalisation") == 0) {
                if (doc["difficulte"].is<int>()) {
                    difficulty = doc["difficulte"];
                    temps[difficulty]  = doc["temps"];
                    force[difficulty]  = doc["force"];
                    speed[difficulty]  = doc["vitesse"];
                }
                if (doc["couleur"].is<const char*>()) {
                    ledColor = doc["couleur"].as<String>();
                }
            }

            // ── REMPLACEMENT ─────────────────────────────────
            else if (strcmp(type, "remplacement") == 0) {
                int32_t valeur = doc["valeur"];

                if      (strcmp(champ, "pince_ouverte")  == 0) OPEN_POS     = valeur;
                else if (strcmp(champ, "pince_fermee")   == 0) CLOSED_POS   = valeur;
                else if (strcmp(champ, "valeurmax_x")    == 0) MAX_POS_X    = valeur;
                else if (strcmp(champ, "valeurmax_y")    == 0) MAX_POS_Y    = valeur;
                else if (strcmp(champ, "valeurmax_z")    == 0) maxDownZPos  = valeur;
                else if (strcmp(champ, "valeurmin_z")    == 0) liftedZPos   = valeur;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

//TODO

//Recevoir les messages par Json
	//Verifier etat Reinitialiser vs Init
	//Gerer bouton urgence

//Fonction Homing not done
//Limit switches in CoreXY
//Probleme frein sur Core XY, du aux boutons
//Probleme pour sortir du IDLE, bouton ok semble ne pas marcher
//Ajouter interface retro
