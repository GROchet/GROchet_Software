#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "event_groups.h"
#include <AccelStepper.h>
#include <ArduinoJson.h>
#include <Dynamixel2Arduino.h>
#include <Adafruit_NeoPixel.h>

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

// Global variables (only motor task writes)
volatile bool btnUp = false;
volatile bool btnDown = false;
volatile bool btnLeft = false;
volatile bool btnRight = false;

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
 
long posX = 0;
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
long liftedZPos = 10000; //Position de l'axe Z quand la pince est levée, à ajuster selon le système
long maxDownZPos = 6000; //Position de l'axe Z quand la pince est au plus bas, à ajuster selon le système

AccelStepper MOT_A = AccelStepper(AccelStepper::DRIVER, STEP_PIN_M1,DIR_PIN_M1); //Moteur gauche
AccelStepper MOT_B = AccelStepper(AccelStepper::DRIVER, STEP_PIN_M2,DIR_PIN_M2); //Moteur droite
AccelStepper MOT_Z = AccelStepper(AccelStepper::DRIVER, STEP_PIN_MZ,DIR_PIN_MZ); //Moteur Z

volatile bool manualControlEnabled = false; // Flag pour indiquer si le contrôle manuel est actif

//--------------------
// Interface utilisateur
//--------------------

#define PIN 5
#define NUMPIXELS 182 * 2
#define DELAY 0
#define largeur 14
#define hauteur 13
#define largeurLettre 4

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
uint32_t BLANC = pixels.Color(15, 15, 15);
uint32_t ROUGE = pixels.Color(15, 0, 0);
uint32_t JAUNE = pixels.Color(15, 15, 0);
uint32_t ORANGE = pixels.Color(20, 5, 0);

// Transformation des lettres et chiffres en matrice 5x3
// Nombre de caractères, nombre de lignes par lettre et nombre de colonnes par lettre
byte alphabet[39][5][3] = {
    {// A
     {1, 1, 1},
     {1, 0, 1},
     {1, 1, 1},
     {1, 0, 1},
     {1, 0, 1}},
 
    {// B
     {1, 1, 1},
     {1, 0, 1},
     {1, 1, 1},
     {1, 0, 1},
     {1, 1, 1}},
 
    {// C
     {1, 1, 1},
     {1, 0, 0},
     {1, 0, 0},
     {1, 0, 0},
     {1, 1, 1}},
 
    {// D
     {1, 1, 0},
     {1, 0, 1},
     {1, 0, 1},
     {1, 0, 1},
     {1, 1, 0}},
 
    {// E
     {1, 1, 1},
     {1, 0, 0},
     {1, 1, 1},
     {1, 0, 0},
     {1, 1, 1}},
 
    {// F
     {1, 1, 1},
     {1, 0, 0},
     {1, 1, 1},
     {1, 0, 0},
     {1, 0, 0}},
 
    {// G
     {1, 1, 1},
     {1, 0, 0},
     {1, 1, 1},
     {1, 0, 1},
     {1, 1, 1}},
 
    {// H
     {1, 0, 1},
     {1, 0, 1},
     {1, 1, 1},
     {1, 0, 1},
     {1, 0, 1}},
 
    {// I
     {1, 1, 1},
     {0, 1, 0},
     {0, 1, 0},
     {0, 1, 0},
     {1, 1, 1}},
 
    {// J
     {1, 1, 1},
     {0, 1, 0},
     {0, 1, 0},
     {0, 1, 0},
     {1, 1, 0}},
 
    {// K
     {1, 0, 1},
     {1, 0, 1},
     {1, 1, 0},
     {1, 0, 1},
     {1, 0, 1}},
 
    {// L
     {1, 0, 0},
     {1, 0, 0},
     {1, 0, 0},
     {1, 0, 0},
     {1, 1, 1}},
 
    {// M
     {1, 0, 1},
     {1, 1, 1},
     {1, 0, 1},
     {1, 0, 1},
     {1, 0, 1}},
 
    {// N
     {1, 1, 1},
     {1, 0, 1},
     {1, 0, 1},
     {1, 0, 1},
     {1, 0, 1}},
 
    {// O
     {0, 1, 0},
     {1, 0, 1},
     {1, 0, 1},
     {1, 0, 1},
     {0, 1, 0}},
 
    {// P
     {1, 1, 1},
     {1, 0, 1},
     {1, 1, 1},
     {1, 0, 0},
     {1, 0, 0}},
 
    {// Q
     {1, 1, 1},
     {1, 0, 1},
     {1, 1, 1},
     {0, 0, 1},
     {0, 0, 1}},
 
    {// R
     {1, 1, 1},
     {1, 0, 1},
     {1, 1, 1},
     {1, 1, 0},
     {1, 0, 1}},
 
    {// S
     {1, 1, 1},
     {1, 0, 0},
     {1, 1, 1},
     {0, 0, 1},
     {1, 1, 1}},
 
    {// T
     {1, 1, 1},
     {0, 1, 0},
     {0, 1, 0},
     {0, 1, 0},
     {0, 1, 0}},
 
    {// U
     {1, 0, 1},
     {1, 0, 1},
     {1, 0, 1},
     {1, 0, 1},
     {1, 1, 1}},
 
    {// V
     {1, 0, 1},
     {1, 0, 1},
     {1, 0, 1},
     {1, 0, 1},
     {0, 1, 0}},
 
    {// W
     {1, 0, 1},
     {1, 0, 1},
     {1, 0, 1},
     {1, 1, 1},
     {1, 0, 1}},
 
    {// X
     {1, 0, 1},
     {1, 0, 1},
     {0, 1, 0},
     {1, 0, 1},
     {1, 0, 1}},
 
    {// Y
     {1, 0, 1},
     {1, 0, 1},
     {0, 1, 0},
     {0, 1, 0},
     {0, 1, 0}},
 
    {// Z
     {1, 1, 1},
     {0, 0, 1},
     {0, 1, 0},
     {1, 0, 0},
     {1, 1, 1}},
 
    {// 0
     {1, 1, 1},
     {1, 0, 1},
     {1, 0, 1},
     {1, 0, 1},
     {1, 1, 1}},
 
    {// 1
     {0, 1, 0},
     {0, 1, 0},
     {0, 1, 0},
     {0, 1, 0},
     {0, 1, 0}},
 
    {// 2
     {1, 1, 1},
     {0, 0, 1},
     {1, 1, 1},
     {1, 0, 0},
     {1, 1, 1}},
 
    {// 3
     {1, 1, 1},
     {0, 0, 1},
     {1, 1, 1},
     {0, 0, 1},
     {1, 1, 1}},
 
    {// 4
     {1, 0, 1},
     {1, 0, 1},
     {1, 1, 1},
     {0, 0, 1},
     {0, 0, 1}},
 
    {// 5
     {1, 1, 1},
     {1, 0, 0},
     {1, 1, 1},
     {0, 0, 1},
     {1, 1, 1}},
 
    {// 6
     {1, 1, 1},
     {1, 0, 0},
     {1, 1, 1},
     {1, 0, 1},
     {1, 1, 1}},
 
    {// 7
     {1, 1, 1},
     {0, 0, 1},
     {0, 0, 1},
     {0, 0, 1},
     {0, 0, 1}},
 
    {// 8
     {1, 1, 1},
     {1, 0, 1},
     {1, 1, 1},
     {1, 0, 1},
     {1, 1, 1}},
 
    {// 9
     {1, 1, 1},
     {1, 0, 1},
     {1, 1, 1},
     {0, 0, 1},
     {0, 0, 1}},
 
    {// !
     {1, 0, 0},
     {1, 0, 0},
     {1, 0, 0},
     {0, 0, 0},
     {1, 0, 0}},
 
    {// :
     {0, 0, 0},
     {0, 1, 0},
     {0, 0, 0},
     {0, 1, 0},
     {0, 0, 0}},
 
    {// /
     {0, 0, 1},
     {0, 0, 0},
     {0, 1, 0},
     {0, 0, 0},
     {1, 0, 0}},
 
};

enum RetroUIState{
    ECRAN_ACCUEIL,
    SELECTION_DIFFICULTE,
    ECRAN_PERDANT,
    ECRAN_GAGNANT,
    RIEN
};
volatile RetroUIState retroUIState = ECRAN_ACCUEIL;

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
  ACCUEIL,
};

volatile SystemState currentState = ACCUEIL;

//Taches et fonctions
void TaskMotorControl (void *pvParameters);
void TaskStateControl (void *pvParameters);
void TaskCommJsonReceive (void *pvParameters);
void TaskCommJsonSend(void *pvParameters);

TaskHandle_t hMotorTask = NULL;
TaskHandle_t hRetroUI   = NULL;
TaskHandle_t hCommSend  = NULL;
TaskHandle_t hCommRecv  = NULL;

#define RX_BUF_SIZE 512

static char rxBuf[RX_BUF_SIZE];
static volatile uint16_t rxHead = 0;
static volatile uint16_t rxTail = 0;

void NotifySwitch();
void NotifyOkButton();
void homeXY();
void ouvrirPince();
void fermerPince();
void sendFullSnapshot();

int transformationIntermediaire(int x, int y);
int transformationCoordonnees(int x, int y);
void allumeLED(int x, int y, uint32_t couleur);
void ecrireLettre(byte matriceLettre[5][3], int xDepart, int yDepart, uint32_t couleur);
int trouverIndexAlphabet(char c);
void ecrireMot(const char *mot, int xDepart, int yDepart, uint32_t couleur);
int longueurMot(const char *mot);
void defilerTexte(const char *mot, int yDepart, uint32_t couleur);
void ecranAccueil(const char *mot);
void processJson(char *incoming);

void EcranAccueil();
void EcranPerdant();
void EcranGagnant();
void EcranDiff();
void TaskRetroUI(void *pvParameters);

static inline void rxBufferPush(char c);
static inline bool rxBufferPop(char &c);

//Global variables for RTOS synchronization
EventGroupHandle_t inputEventGroup;

volatile bool jsonMoveActive = false;

void setup() {
	Serial.begin(115200);
	delay(2000);

    sendFullSnapshot();
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
	MOT_Z.setMaxSpeed(2000);

	MOT_A.setAcceleration(2000);
	MOT_B.setAcceleration(2000);
	MOT_Z.setAcceleration(500);

	MOT_A.setCurrentPosition(0);
	MOT_B.setCurrentPosition(0);
	MOT_Z.setCurrentPosition(0);

    pixels.begin();
    pixels.clear();
    pixels.show();

	//RTOS
	//----------------
	inputEventGroup = xEventGroupCreate();
	limitEventGroup = xEventGroupCreate();

	xTaskCreate(TaskMotorControl, "MotorTask", 256, NULL, 3, &hMotorTask);
	xTaskCreate(TaskStateControl, "StateTask", 512, NULL, 4, NULL);
	xTaskCreate(TaskCommJsonSend,    "CommSend", 513, NULL, 3, &hCommSend);
	xTaskCreate(TaskCommJsonReceive, "CommRecv", 512, NULL, 3, &hCommRecv);
    //xTaskCreate(TaskRetroUI, "RetroUI", 256, NULL, 3, &hRetroUI); //128 is ok, we chose 256 to be safe
}

void loop() {
}

void homeXY() {
    
    MOT_A.stop();
    MOT_B.stop();
 
    // ── Compute XY target based on currentPosition ──
    // --- Home X axis ---
    while (digitalRead(LMTSW_X) != LOW) {
        long curX = (MOT_A.currentPosition() + MOT_B.currentPosition()) / 2;
        long curY = (MOT_A.currentPosition() - MOT_B.currentPosition()) / 2;
 
        posX = curX;
        posY = curY;
 
        // Down
        posX -= 100;
 
        MOT_A.moveTo(posX + posY);
        MOT_B.moveTo(posX - posY);
        vTaskDelay(pdMS_TO_TICKS(15));
    }
 
    MOT_A.setCurrentPosition(0);
    MOT_B.setCurrentPosition(0);
    posX = 0;
 
    // --- Home Y axis ---
    while (digitalRead(LMTSW_Y) != LOW) {
        long curX = (MOT_A.currentPosition() + MOT_B.currentPosition()) / 2;
        long curY = (MOT_A.currentPosition() - MOT_B.currentPosition()) / 2;
 
        posX = curX;
        posY = curY;
 
        // Left
        posY += 100;
 
        MOT_A.moveTo(posX + posY);
        MOT_B.moveTo(posX - posY);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
 
    MOT_A.setCurrentPosition(0);
    MOT_B.setCurrentPosition(0);
    posY = 0;
}

void TaskStateControl (void *pvParameters) {
  (void) pvParameters;

  for(;;) {

	switch(currentState) {
        case ACCUEIL:
            //vTaskSuspend(hCommRecv);
            vTaskSuspend(hMotorTask); // Suspendre la tâche de contrôle des moteurs pendant l'écran d'accueil
            retroUIState = ECRAN_ACCUEIL;
            xEventGroupWaitBits(inputEventGroup, EVT_BTN_OK, pdTRUE, pdFALSE, portMAX_DELAY);
            currentState = SETUP;
            break;
        case SETUP:
            //vTaskResume(hCommRecv); // Reprendre la tâche de communication pour recevoir les données de configuration pendant le setup
            vTaskResume(hMotorTask);
            retroUIState = RIEN;
            xEventGroupWaitBits(inputEventGroup, EVT_BTN_OK, pdTRUE, pdFALSE, portMAX_DELAY);
            ouvrirPince();
            homeXY();
            //Remonter axe Z à ajouter
			currentState = DIFF_CHOOSE;
			break;
        case DIFF_CHOOSE:
            //vTaskSuspend(hCommRecv);
            vTaskSuspend(hMotorTask); // Suspendre la tâche de contrôle des moteurs pendant le choix de la difficulté
            retroUIState = SELECTION_DIFFICULTE;
            vTaskResume(hRetroUI); // S'assurer que la tâche de l'interface utilisateur est active pour afficher le menu de sélection
            
            xEventGroupWaitBits(inputEventGroup, EVT_BTN_OK, pdTRUE, pdFALSE, portMAX_DELAY);

            retroUIState = RIEN;
            currentState = IDLE;
            break;
	    case IDLE:
            //vTaskResume(hCommRecv); // Reprendre la tâche de communication pour recevoir les commandes de mouvement pendant l'état IDLE
            vTaskResume(hMotorTask); // Reprendre la tâche de contrôle des moteurs une fois la difficulté choisie
            manualControlEnabled = true; // Allow manual control in TaskMotorControl
            xEventGroupWaitBits(inputEventGroup, EVT_BTN_OK, pdTRUE, pdFALSE, portMAX_DELAY);
            manualControlEnabled = false; // Disable manual control
            currentState = LOWERING;
			break;
	    case LOWERING:
			//Séquence de mouvement vers le bas
            while(1){ // Tant que l'axe Z n'est pas presque au plus bas
                MOT_Z.moveTo(MOT_Z.currentPosition() - 100) ; // Update internal position
                if(xEventGroupWaitBits(inputEventGroup, EVT_BTN_OK, pdTRUE, pdFALSE, pdMS_TO_TICKS(20)) || abs(MOT_Z.currentPosition()-maxDownZPos) < 50) {
                    MOT_Z.stop();
                    break; // Sortir de la boucle pour arrêter la descente
                }
                vTaskDelay(pdMS_TO_TICKS(40));
            }
            vTaskSuspend(hMotorTask); // Suspendre la tâche de contrôle des moteurs pendant que la pince est fermée
			currentState = CLOSING;
			break;

	    case CLOSING:{  
			fermerPince();
			currentState = LIFTING;
			break;
	    }
	    case LIFTING:
			//WAIT FOR Z TO BE LIFTED, THEN MOVE TO DROPZONE
            vTaskResume(hMotorTask); // S'assurer que la tâche de communication est active pour envoyer les mises à jour de position pendant le levage
			MOT_Z.moveTo(liftedZPos);
            while(abs(MOT_Z.currentPosition() - liftedZPos) > 50) {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
			currentState = MOVING_TO_DROPZONE;
			break;

	    case MOVING_TO_DROPZONE: //DONE
			//Séquence de mouvement vers la dropzone
			homeXY();
			currentState = DROPPING;
			break;

	    case DROPPING:{ //DONE;
			ouvrirPince();
			currentState = IDLE;
			break;
	    }
	}
	vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void TaskCommJsonSend(void *pvParameters) {
    (void) pvParameters;

    struct {
        long posX = -999999;
        long posY = -999999;
        long zPos = -999999;
        int state = -1;
        int ACTUAL_POS = -1;
        int difficulty = -1;

        int temps[3] = {-1,-1,-1};
        int force[3] = {-1,-1,-1};
        int speed[3] = {-1,-1,-1};

        int OPEN_POS = -1;
        int CLOSED_POS = -1;

        long MAX_POS_X = -1;
        long MAX_POS_Y = -1;
        long liftedZPos = -1;
        long maxDownZPos = -1;

        String ledColor = "";

        bool btnUp = false;
        bool btnDown = false;
        bool btnLeft = false;
        bool btnRight = false;
        bool btnOk = false;
    } last;

    for (;;) {

        bool send = false;

        StaticJsonDocument<768> doc;

        // ---------------- POSITION ----------------
        if (posX != last.posX) {
            doc["posX"] = posX;
            last.posX = posX;
            send = true;
        }

        if (posY != last.posY) {
            doc["posY"] = posY;
            last.posY = posY;
            send = true;
        }

        long z = MOT_Z.currentPosition();
        if (z != last.zPos) {
            doc["zPos"] = z;
            last.zPos = z;
            send = true;
        }

        // ---------------- STATE ----------------
        if ((int)currentState != last.state) {
            doc["state"] = (int)currentState;
            last.state = (int)currentState;
            send = true;
        }

        if (ACTUAL_POS != last.ACTUAL_POS) {
            doc["pos_act"] = ACTUAL_POS;
            last.ACTUAL_POS = ACTUAL_POS;
            send = true;
        }

        if (difficulty != last.difficulty) {
            doc["diff"] = difficulty;
            last.difficulty = difficulty;
            send = true;
        }

        // ---------------- ARRAYS ----------------
        bool tempsChanged = false;
        bool forceChanged = false;
        bool speedChanged = false;
        for (int i = 0; i < 3; i++) {
            if (temps[i] != last.temps[i]) {
                tempsChanged = true;
            }
            if (force[i] != last.force[i]) {
                forceChanged = true;
            }
            if (speed[i] != last.speed[i]) {
                speedChanged = true;
            }
        }
        if (tempsChanged) {
            JsonArray arrTemps = doc.createNestedArray("temps");
            for (int i = 0; i < 3; i++) {
                arrTemps.add(temps[i]);
                last.temps[i] = temps[i];
            }
            send = true;
        }
        
        if (forceChanged) {
            JsonArray arrForce = doc.createNestedArray("force");
            for (int i = 0; i < 3; i++) {
                arrForce.add(force[i]);

                last.force[i] = force[i];

            }

            send = true;

        }
        
        if (speedChanged) {

            JsonArray arrSpeed = doc.createNestedArray("speed");

            for (int i = 0; i < 3; i++) {

                arrSpeed.add(speed[i]);

                last.speed[i] = speed[i];

            }

            send = true;

        }
        

        // ---------------- PINCE ----------------
        if (OPEN_POS != last.OPEN_POS) {
            doc["pince_open"] = OPEN_POS;
            last.OPEN_POS = OPEN_POS;
            send = true;
        }

        if (CLOSED_POS != last.CLOSED_POS) {
            doc["pince_closed"] = CLOSED_POS;
            last.CLOSED_POS = CLOSED_POS;
            send = true;
        }

        // ---------------- LIMITS ----------------
        if (MAX_POS_X != last.MAX_POS_X) {
            doc["max_x"] = MAX_POS_X;
            last.MAX_POS_X = MAX_POS_X;
            send = true;
        }

        if (MAX_POS_Y != last.MAX_POS_Y) {
            doc["max_y"] = MAX_POS_Y;
            last.MAX_POS_Y = MAX_POS_Y;
            send = true;
        }

        if (liftedZPos != last.liftedZPos) {
            doc["z_high"] = liftedZPos;
            last.liftedZPos = liftedZPos;
            send = true;
        }

        if (maxDownZPos != last.maxDownZPos) {
            doc["z_down"] = maxDownZPos;
            last.maxDownZPos = maxDownZPos;
            send = true;
        }

        // ---------------- LED ----------------
        if (ledColor != last.ledColor) {
            doc["led"] = ledColor;
            last.ledColor = ledColor;
            send = true;
        }

        // ---------------- BUTTONS ----------------
        bool up    = btnUp;
        bool down  = btnDown;
        bool left  = btnLeft;
        bool right = btnRight;
        bool ok    = (xEventGroupGetBits(inputEventGroup) & EVT_BTN_OK);

        if (up != last.btnUp) {
            doc["btn_up"] = up;
            last.btnUp = up;
            send = true;
        }

        if (down != last.btnDown) {
            doc["btn_down"] = down;
            last.btnDown = down;
            send = true;
        }

        if (left != last.btnLeft) {
            doc["btn_left"] = left;
            last.btnLeft = left;
            send = true;
        }

        if (right != last.btnRight) {
            doc["btn_right"] = right;
            last.btnRight = right;
            send = true;
        }

        if (ok != last.btnOk) {
            doc["btn_ok"] = ok;
            last.btnOk = ok;
            send = true;
        }

        // ---------------- SEND ONLY IF CHANGED ----------------
        if (send) {
            serializeJson(doc, Serial);
            Serial.print('\n');
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void sendFullSnapshot() {
    StaticJsonDocument<1024> doc;

    doc["type"] = "full";

    doc["posX"] = posX;
    doc["posY"] = posY;
    doc["zPos"] = MOT_Z.currentPosition();
    doc["state"] = (int)currentState;
    doc["diff"] = difficulty;

    // --- Pince ---
    JsonObject pince = doc.createNestedObject("pince");
    pince["pos_o"] = OPEN_POS;
    pince["pos_f"] = CLOSED_POS;
    pince["pos_act"] = ACTUAL_POS;

    // --- Limits ---
    JsonObject limits = doc.createNestedObject("limits");
    limits["maxPosX"] = MAX_POS_X;
    limits["maxPosY"] = MAX_POS_Y;
    limits["maxH"] = liftedZPos;
    limits["minH"] = maxDownZPos;

    // --- Arrays ---
    doc["temps"][0] = temps[0];
    doc["temps"][1] = temps[1];
    doc["temps"][2] = temps[2];

    doc["force"][0] = force[0];
    doc["force"][1] = force[1];
    doc["force"][2] = force[2];

    doc["speed"][0] = speed[0];
    doc["speed"][1] = speed[1];
    doc["speed"][2] = speed[2];

    doc["ledColor"] = ledColor;

    // --- Buttons ---
    JsonObject buttons = doc.createNestedObject("buttons");
    buttons["haut"]   = btnUp;
    buttons["bas"]    = btnDown;
    buttons["gauche"] = btnLeft;
    buttons["droite"] = btnRight;
    buttons["ok"]     = false;

    serializeJson(doc, Serial);
    Serial.print('\n');
}

void TaskMotorControl(void *pvParameters) {
    (void) pvParameters;

    const long btnIncrement = 100; // increment for buttons, can also use speed[difficulty]

    for (;;) {
        if(manualControlEnabled){
            // ── Read buttons ──
            bool up    = digitalRead(BTN_PIN_UP)    == LOW;
            bool down  = digitalRead(BTN_PIN_DOWN)  == LOW;
            bool left  = digitalRead(BTN_PIN_LEFT)  == LOW;
            bool right = digitalRead(BTN_PIN_RIGHT) == LOW;

            btnUp = up; btnDown = down; btnLeft = left; btnRight = right;
            
            if(btnUp || btnDown || btnLeft || btnRight) {
                // ── Compute XY target based on currentPosition ──
                long curX = (MOT_A.currentPosition() + MOT_B.currentPosition()) / 2;
                long curY = (MOT_A.currentPosition() - MOT_B.currentPosition()) / 2;

                long targetX = curX;
                long targetY = curY;

                if (up)    targetX += btnIncrement;
                if (down)  targetX -= btnIncrement;
                if (left)  targetY += btnIncrement; 
                if (right) targetY -= btnIncrement; 

                // ── Set moveTo targets if there is any movement ──
                posX = targetX; 
                posY = targetY; 
                MOT_A.moveTo(targetX + targetY);
                MOT_B.moveTo(targetX - targetY);
            }
        }
        // ── Run motors periodically ──
        bool motorsActive = abs(MOT_A.distanceToGo()) > 1 ||
                    abs(MOT_B.distanceToGo()) > 1 ||
                    abs(MOT_Z.distanceToGo()) > 1;

        if (motorsActive) {
            MOT_A.run();
            MOT_B.run();
            MOT_Z.run();
        }
        vTaskDelay(pdMS_TO_TICKS(15));
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
    if ((now - lastBtnInterrupt < pdMS_TO_TICKS(DEBOUNCE_MS)) || digitalRead(BTN_PIN_OK) == HIGH) return;
    lastBtnInterrupt = now;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (digitalRead(BTN_PIN_OK) == LOW)
    xEventGroupSetBitsFromISR(inputEventGroup, EVT_BTN_OK, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

void ouvrirPince() {
    dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, MOVE_CURRENT);
    dxl.setGoalPosition(id, OPEN_POS, UNIT_RAW);
    while (ACTUAL_POS > OPEN_POS + 50) { // Tant que la pince n'est pas presque ouverte
        vTaskDelay(pdMS_TO_TICKS(50));
        ACTUAL_POS = dxl.getPresentPosition(id);
    }
}

void fermerPince() {
    dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, GRIP_CURRENT);
    dxl.setGoalPosition(id, CLOSED_POS, UNIT_RAW);
    while (ACTUAL_POS < CLOSED_POS - 50) { // Tant que la pince n'est pas presque fermée
        vTaskDelay(pdMS_TO_TICKS(50));
        ACTUAL_POS = dxl.getPresentPosition(id);
    }
}

void processJson(char *incoming) {
    StaticJsonDocument<768> doc;
 
    DeserializationError err = deserializeJson(doc, incoming);
    
    if (err) {
        /*
        Serial.print("JSON ERROR: ");
        Serial.println(err.c_str());
        Serial.print("JSON LEN = ");
        Serial.println(strlen(incoming));
        Serial.println((uint8_t)incoming[0], HEX);
        Serial.println((uint8_t)incoming[1], HEX);*/
        return;
    }
 
    const char* type = doc["type"];
    if (!type) {
        return;
    }
 
    if (strcmp(type, "commande") == 0) {
        const char* action = doc["action"];
        if (!action) return;
 
        if (strcmp(action, "urgence") == 0) {
            MOT_A.stop();
            MOT_B.stop();
            MOT_Z.stop();
            while (1) {}
        }
        else if (strcmp(action, "reinitialiser") == 0) {
            currentState = SETUP;
        }
        else if (strcmp(action, "init") == 0) {
            homeXY();
        }
        else if (strcmp(action, "ouvrir_pince") == 0) {
            ouvrirPince();
        }
        else if (strcmp(action, "fermer_pince") == 0) {
            fermerPince();
        }
        else if (strcmp(action, "moitie_pince") == 0) {
            int target = (OPEN_POS + CLOSED_POS) / 2;
            dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, GRIP_CURRENT);
            dxl.setGoalPosition(id, target, UNIT_RAW);
 
            while (abs(ACTUAL_POS - target) > 50) {
                vTaskDelay(pdMS_TO_TICKS(20));
                ACTUAL_POS = dxl.getPresentPosition(id);
            }
        }
        else if (
            strcmp(action, "dep_droite") == 0 ||
            strcmp(action, "dep_gauche") == 0 ||
            strcmp(action, "dep_haut") == 0 ||
            strcmp(action, "dep_bas") == 0
        ) {
            long curX = (MOT_A.currentPosition() + MOT_B.currentPosition()) / 2;
            long curY = (MOT_A.currentPosition() - MOT_B.currentPosition()) / 2;
 
            long dx = 0;
            long dy = 0;
 
            if (strcmp(action, "dep_droite") == 0) dy = -200;
            if (strcmp(action, "dep_gauche") == 0) dy = 200;
            if (strcmp(action, "dep_haut") == 0) dx = 200;
            if (strcmp(action, "dep_bas") == 0) dx = -200;
 
            posX = curX + dx;
            posY = curY + dy;
            MOT_A.moveTo(posX + posY);
            MOT_B.moveTo(posX - posY);
            jsonMoveActive = true;
        }
        else if (strcmp(action, "dep_z_haut") == 0) {
            MOT_Z.moveTo(MOT_Z.currentPosition() - 200);
            jsonMoveActive = true;
        }
        else if (strcmp(action, "dep_z_bas") == 0) {
            MOT_Z.moveTo(MOT_Z.currentPosition() + 200);
            jsonMoveActive = true;
        }
        else if (strcmp(action, "pos_haut_z") == 0) {
            MOT_Z.moveTo(liftedZPos);
            jsonMoveActive = true;
        }
        else if (strcmp(action, "pos_bas_z") == 0) {
            MOT_Z.moveTo(maxDownZPos);
            jsonMoveActive = true;
        }
        else if (strcmp(action, "pos_milieu_xy") == 0) {
            posX = MAX_POS_X / 2;
            posY = MAX_POS_Y / 2;
            MOT_A.moveTo(posX + posY);
            MOT_B.moveTo(posX - posY);
            jsonMoveActive = true;
        }
        else if (strcmp(action, "ouvrir_manuel") == 0) {
            int32_t target = dxl.getPresentPosition(id) - 200;
            dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, MOVE_CURRENT);
            dxl.setGoalPosition(id, target, UNIT_RAW);
 
            while (ACTUAL_POS > target + 20) {
                vTaskDelay(pdMS_TO_TICKS(20));
                ACTUAL_POS = dxl.getPresentPosition(id);
            }
        }
        else if (strcmp(action, "fermer_manuel") == 0) {
            int32_t target = dxl.getPresentPosition(id) + 200;
            dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, MOVE_CURRENT);
            dxl.setGoalPosition(id, target, UNIT_RAW);
 
            while (ACTUAL_POS < target - 20) {
                vTaskDelay(pdMS_TO_TICKS(20));
                ACTUAL_POS = dxl.getPresentPosition(id);
            }
        }
 
        return;
    }
 
    if (strcmp(type, "pers") == 0) {
        if (doc["clr"].is<const char*>()) {
            ledColor = doc["clr"].as<const char*>();
        }
 
        if (doc["diff"].is<const char*>()) {
            const char* diff = doc["diff"];
 
            if (strcmp(diff, "fac") == 0) difficulty = 0;
            else if (strcmp(diff, "moy") == 0) difficulty = 1;
            else if (strcmp(diff, "exp") == 0) difficulty = 2;
 
            temps[difficulty] = doc["t"] | temps[difficulty];
            force[difficulty] = doc["F"] | force[difficulty];
            speed[difficulty] = doc["v"] | speed[difficulty];
        }
 
        return;
    }
 
    if (strcmp(type, "rep") == 0) {
        const char* champ = doc["champ"];
        int32_t valeur = doc["valeur"] | 0;
        if (!champ) return;
 
        if (strcmp(champ, "pince_ouverte") == 0) OPEN_POS = valeur;
        else if (strcmp(champ, "pince_fermee") == 0) CLOSED_POS = valeur;
        else if (strcmp(champ, "valeurmax_x") == 0) MAX_POS_X = valeur;
        else if (strcmp(champ, "valeurmax_y") == 0) MAX_POS_Y = valeur;
        else if (strcmp(champ, "valeurmax_z") == 0) liftedZPos = valeur;
        else if (strcmp(champ, "valeurmin_z") == 0) maxDownZPos = valeur;
 
        return;
    }
}

static inline void rxBufferPush(char c) {
    uint16_t next = (rxHead + 1) % RX_BUF_SIZE;

    // overflow protection: drop oldest data
    if (next == rxTail) {
        rxTail = (rxTail + 1) % RX_BUF_SIZE;
    }

    rxBuf[rxHead] = c;
    rxHead = next;
}

static inline bool rxBufferPop(char &c) {
    if (rxTail == rxHead) return false;

    c = rxBuf[rxTail];
    rxTail = (rxTail + 1) % RX_BUF_SIZE;
    return true;
}

void TaskCommJsonReceive(void *pvParameters) {
    (void) pvParameters;

    static char lineBuf[512];
    static uint16_t lineLen = 0;

    for (;;) {

        // 1. READ SERIAL → RING BUFFER ONLY
        while (Serial.available()) {
            char c = Serial.read();
            rxBufferPush(c);
        }

        // 2. PARSE LINES FROM BUFFER
        char c;
        while (rxBufferPop(c)) {

            if (c == '\r') continue;

            if (c == '\n') {
                lineBuf[lineLen] = '\0';
                if (lineLen > 0) processJson(lineBuf);
                lineLen = 0;
            } else {
                if (lineLen < sizeof(lineBuf) - 1) {
                    lineBuf[lineLen++] = c;
                } else {
                    lineLen = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}


//----------------------
// interface Utilisateur
//----------------------

// Fonction qui transforme des coordonnées de la matrice 13x14 en numéro de LED pour la librairie NeoPixel
int transformationIntermediaire(int x, int y){
  int indexIntermediaire = (y * largeur) + x;
 
  return indexIntermediaire;
};
 
// Fonction qui transforme des coordonnées de la matrice LED complète (13x28) en numéro de LED pour la librairie NeoPixel
int transformationCoordonnees(int x, int y){
  int index = 0;
  if (x < largeur)
  {
    index = transformationIntermediaire(x, y);
  }
 
  else
  {
    int xLocal = x - largeur;
    index = transformationIntermediaire(xLocal, y) + (largeur * hauteur);
  }
 
  return index;
};
 
// Fonction qui allume d'une certaine couleur la LED correspondant aux coordonnées de la matrice 13x28
// La variable couleur contient pixels.Color(R,G,B)
void allumeLED(int x, int y, uint32_t couleur){
  if (x >= 0 && x < 28 && y >= 0 && y < 13)
  {
    int index = transformationCoordonnees(x, y);
 
    pixels.setPixelColor(index, couleur);
  }
};
 
void ecrireLettre(byte matriceLettre[5][3], int xDepart, int yDepart, uint32_t couleur){
 
  for (int rangee = 0; rangee < 5; rangee++)
  {
 
    for (int colonne = 0; colonne < 3; colonne++)
    {
 
      if (matriceLettre[rangee][colonne] == 1)
      {
        allumeLED(xDepart + colonne, yDepart + rangee, couleur);
      }
    }
  }
};
 
// Fonction qui trouve la lettre dans la matrice 3D de l'alphabet
int trouverIndexAlphabet(char c){
  int index = 0;
  if (c >= 'A' && c <= 'Z')
  {
    index = c - 'A';
  }
 
  else if (c >= '0' && c <= '9')
  {
    index = 26 + c - '0';
  }
 
  else if (c == '!')
  {
    index = 36;
  }
 
  else if (c == ':')
  {
    index = 37;
  }
 
  else if (c == '/')
  {
    index = 38;
  }
 
  else
  {
    index = -1;
  }
 
  return index;
};
 
void ecrireMot(const char *mot, int xDepart, int yDepart, uint32_t couleur){
  for (int i = 0; mot[i] != '\0'; i++)
  {
    char lettre = mot[i];
    int index = trouverIndexAlphabet(lettre);
    if (index != -1)
    {
      ecrireLettre(alphabet[index], (xDepart + i * largeurLettre), yDepart, couleur);
    }
  }
}
 
int longueurMot(const char *mot){
  int longueur = 0;
  while (mot[longueur] != '\0')
  {
    longueur++;
  }
  return longueur;
}
 
void defilerTexte(const char *mot, int yDepart, uint32_t couleur){
  static int x = 28;
 
  int largeurMot = longueurMot(mot) * 4;
 
  ecrireMot(mot, x, yDepart, couleur);
 
  x--;
 
  if (x < -largeurMot)
  {
    x = 28;
  }
}

void TaskRetroUI(void *pvParameters){
    (void) pvParameters;
    for(;;){
        switch(retroUIState){
            case ECRAN_ACCUEIL:
                EcranAccueil();
                break;
            case SELECTION_DIFFICULTE:
                EcranDiff();
                break;
            case ECRAN_GAGNANT:
                EcranGagnant();
                break;
            case ECRAN_PERDANT:
                EcranPerdant();
                break;
            case RIEN:
                [[fallthrough]];
            default:
                pixels.clear();
                pixels.show();
                vTaskSuspend(NULL);
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void EcranAccueil(){
    static int decalage = 0;
    pixels.clear();
    for (int i = -3; i < 28; i += 3){
        allumeLED(i + decalage, 1, pixels.Color(25, 0, 25));
        allumeLED(i + 1 + decalage, 1, pixels.Color(25, 10, 0));
        allumeLED(i + 2 + decalage, 1, pixels.Color(0, 5, 15));
        allumeLED(i + decalage, 11, pixels.Color(25, 0, 25));
        allumeLED(i + 1 + decalage, 11, pixels.Color(25, 10, 0));
        allumeLED(i + 2 + decalage, 11, pixels.Color(0, 5, 15));
    }

    ecrireMot("GROCHET", 0, 4, pixels.Color(10, 40, 8));
    pixels.show();
    vTaskDelay(pdMS_TO_TICKS(200));
    decalage++;
    if (decalage == 3) decalage = 0;
}

void EcranPerdant(){
    int largeurMot = longueurMot("MEILLEURE CHANCE LA PROCHAINE FOIS") * 4;
    for (int i = 28; i > -(largeurMot); i--){
        pixels.clear();
        int decalage = 0;
        for (int j = 0; j < 28; j++){
            // Motif
            allumeLED(j * 4 + decalage, 0, pixels.Color(25, 0, 0));
            allumeLED(j * 4 + 2 + decalage, 0, pixels.Color(25, 0, 0));
            allumeLED(j * 4 + 1 + decalage, 1, pixels.Color(25, 0, 0));
            allumeLED(j * 4 + decalage, 2, pixels.Color(25, 0, 0));
            allumeLED(j * 4 + 2 + decalage, 2, pixels.Color(25, 0, 0));

            allumeLED(j * 4 + decalage, 10, pixels.Color(25, 0, 0));
            allumeLED(j * 4 + 2 + decalage, 10, pixels.Color(25, 0, 0));
            allumeLED(j * 4 + 1 + decalage, 11, pixels.Color(25, 0, 0));
            allumeLED(j * 4 + decalage, 12, pixels.Color(25, 0, 0));
            allumeLED(j * 4 + 2 + decalage, 12, pixels.Color(25, 0, 0));
            decalage = decalage + 2;
        }
        ecrireMot("MEILLEURE CHANCE LA PROCHAINE FOIS", i, 4, pixels.Color(0, 0, 25));
        pixels.show();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void EcranGagnant(){
    static bool afficher = true;
    pixels.clear();
    int decalage = 0;
    for (int j = 0; j < 28; j++){
        // Coeur en haut
        allumeLED(0 + decalage, 0, pixels.Color(25, 0, 10));
        allumeLED(1 + decalage, 0, pixels.Color(25, 0, 10));
        allumeLED(3 + decalage, 0, pixels.Color(25, 0, 10));
        allumeLED(4 + decalage, 0, pixels.Color(25, 0, 10));

        allumeLED(0 + decalage, 1, pixels.Color(25, 0, 10));
        allumeLED(1 + decalage, 1, pixels.Color(25, 0, 10));
        allumeLED(2 + decalage, 1, pixels.Color(25, 0, 10));
        allumeLED(3 + decalage, 1, pixels.Color(25, 0, 10));
        allumeLED(4 + decalage, 1, pixels.Color(25, 0, 10));

        allumeLED(1 + decalage, 2, pixels.Color(25, 0, 10));
        allumeLED(2 + decalage, 2, pixels.Color(25, 0, 10));
        allumeLED(3 + decalage, 2, pixels.Color(25, 0, 10));

        allumeLED(2 + decalage, 3, pixels.Color(25, 0, 10));

        // Coeur en bas
        allumeLED(0 + decalage, 9, pixels.Color(25, 0, 10));
        allumeLED(1 + decalage, 9, pixels.Color(25, 0, 10));
        allumeLED(3 + decalage, 9, pixels.Color(25, 0, 10));
        allumeLED(4 + decalage, 9, pixels.Color(25, 0, 10));

        allumeLED(0 + decalage, 10, pixels.Color(25, 0, 10));
        allumeLED(1 + decalage, 10, pixels.Color(25, 0, 10));
        allumeLED(2 + decalage, 10, pixels.Color(25, 0, 10));
        allumeLED(3 + decalage, 10, pixels.Color(25, 0, 10));
        allumeLED(4 + decalage, 10, pixels.Color(25, 0, 10));

        allumeLED(1 + decalage, 11, pixels.Color(25, 0, 10));
        allumeLED(2 + decalage, 11, pixels.Color(25, 0, 10));
        allumeLED(3 + decalage, 11, pixels.Color(25, 0, 10));

        allumeLED(2 + decalage, 12, pixels.Color(25, 0, 10));

        decalage = 23;
    }
    if (afficher) ecrireMot("BRAVO!", 3, 4, pixels.Color(0, 20, 5));

    afficher = !afficher;
    pixels.show();
}

void EcranDiff(){
    static bool prevLeft = false;
    static bool prevRight = false;

    bool left  = digitalRead(BTN_PIN_LEFT)  == LOW;
    bool right = digitalRead(BTN_PIN_RIGHT) == LOW;

    if (left && !prevLeft) {
        difficulty = max(0, difficulty - 1);
    }
    if (right && !prevRight) {
        difficulty = min(2, difficulty + 1);
    }

    prevLeft = left;
    prevRight = right;
    
    pixels.clear();
    defilerTexte("CHOIX DIFFICULTE", 0, BLANC);

    if (difficulty == 0){
        ecrireMot("FACILE", 0, 8, JAUNE);
        // Fleche
        allumeLED(25, 8, BLANC);
        allumeLED(26, 9, BLANC);
        allumeLED(27, 10, BLANC);
        allumeLED(26, 11, BLANC);
        allumeLED(25, 12, BLANC);
    }
    
    else if (difficulty == 1){
        ecrireMot("MOYEN", 0, 8, ORANGE);
        // Fleche
        allumeLED(21, 8, BLANC);
        allumeLED(22, 9, BLANC);
        allumeLED(23, 10, BLANC);
        allumeLED(22, 11, BLANC);
        allumeLED(21, 12, BLANC);
    }
    
    else if (difficulty == 2){
        ecrireMot("EXPERT", 0, 8, ROUGE);
        // Fleche
        allumeLED(25, 8, BLANC);
        allumeLED(26, 9, BLANC);
        allumeLED(27, 10, BLANC);
        allumeLED(26, 11, BLANC);
        allumeLED(25, 12, BLANC);
    }
    pixels.show();
}

//TODO

//Recevoir les messages par Json
	//Verifier etat Reinitialiser vs Init
	//Gerer bouton urgence

//Limit switches in CoreXY

//Ajouter interface retro

//Mettre un bool pour autoriser les boutons qui gerent les moteurs. only dans IDLE

//Fonction bouger pince qui prends en argument la position, et attends : Normaliser fcts fermerPince, ouvrirPince. ajouter motie pince
//Pareil pour axe Z

//Ecran affichage parametre pour UI retro