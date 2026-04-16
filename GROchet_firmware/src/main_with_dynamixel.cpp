#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "event_groups.h"
#include <AccelStepper.h>
#include <ArduinoJson.h>
#include <Dynamixel2Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <avr/pgmspace.h>

//À considérer/Calibrer avant de Run la premiere fois :

// OPEN_POS et CLOSED_POS du dynamixel (pince)
// MAX_POS_X and MAX_POS_Y according to the system's dimensions (CORE_XY)
// maxDownZPos et liftedZPos (position min et max pour axe Z)

//-------------------
//Difficulté
//-------------------
int16_t temps[] = {60,30,20}; //A ajuster selon les tests, en s, pour chaque difficulté (easy, medium, hard). Temps pendant lequel le toutou doit être attrapé pour valider la prise et passer à l'étape suivante. Peut être différent selon la difficulté choisie.
int16_t force[] = {5,2,1}; //A ajuster selon les tests, en unités brutes du dynamixel
int16_t speed[] = {100,150,250}; //A ajuster selon les tests, en unités de vitesse du stepper (peut être différent selon le système et les moteurs utilisés)
int8_t difficulty = 0; // 0 = easy, 1 = medium, 2 = hard. A ajuster selon les tests

enum LedColor {
    LED_ROUGE = 0,
    LED_ROSE, //1
    LED_ORANGE, //2
    LED_BLEU, //3
    LED_VERT, //4
    LED_JAUNE, //5
    LED_MAUVE, //6
    LED_BLANC //7
};
LedColor ledColor = LED_VERT; // Couleur actuelle des LEDs, à ajuster selon les tests

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

#define MOVE_CURRENT    100 // Courant à appliquer pour déplacer la pince (unités brutes)
#define GRIP_CURRENT    5 // Courant à appliquer pour fermer la pince (unités brutes)
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

SemaphoreHandle_t serialMutex;

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
uint32_t BLANC = pixels.Color(15, 15, 15);
uint32_t ROUGE = pixels.Color(15, 0, 0);
uint32_t JAUNE = pixels.Color(15, 15, 0);
uint32_t ORANGE = pixels.Color(20, 5, 0);

const uint8_t ROUGE_l[] = {75, 0, 0};
const uint8_t ROSE_l[] = {75, 0, 75};
const uint8_t ORANGE_l[] = {75, 50, 0};
const uint8_t BLEU_l[] = {0, 55, 75};
const uint8_t VERT_l[] = {0, 75, 0};
const uint8_t JAUNE_l[] = {75, 75, 0};
const uint8_t MAUVE_l[] = {59, 0, 75};
const uint8_t BLANC_l[] = {75, 75, 75};

// Transformation des lettres et chiffres en matrice 5x3
// Nombre de caractères, nombre de lignes par lettre et nombre de colonnes par lettre
const uint8_t alphabet[39][5][3] PROGMEM= {
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
    RIEN,
    TIMER,
};
volatile RetroUIState retroUIState = ECRAN_ACCUEIL;

//----------------------
// LED STRIP
//----------------------
#define PIN_LED 4

#define NUMPIXELS_BANDE 20
#define OFFSET_LED_GAUCHE 0
#define OFFSET_LED_COMPTEUR 40
#define OFFSET_LED_DROIT 100
#define OFFSET_LED_INT 140

static int n_LED_compteur = NUMPIXELS_BANDE*3;
static int n_LED_total = NUMPIXELS_BANDE*11;

Adafruit_NeoPixel pixels_LED(n_LED_total, PIN_LED, NEO_GRB + NEO_KHZ800);


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

static StaticJsonDocument<256> doc_i;
static StaticJsonDocument<512> doc_o;

struct LastState{
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

    int ledColor = -1;

    bool btnUp = false;
    bool btnDown = false;
    bool btnLeft = false;
    bool btnRight = false;
    bool btnOk = false;
};
static LastState last;

//Taches et fonctions
void TaskMotorControl (void *pvParameters);
void TaskStateControl (void *pvParameters);
void TaskCommJsonReceive (void *pvParameters);
void TaskCommJsonSend(void *pvParameters);

TaskHandle_t hMotorTask = NULL;
TaskHandle_t hRetroUI   = NULL;
TaskHandle_t hCommSend  = NULL;
TaskHandle_t hCommRecv  = NULL;

#define RX_BUF_SIZE 128 //128 tanto FOURCHETTE

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
void ecrireLettre(const uint8_t matriceLettre[5][3], int xDepart, int yDepart, uint32_t couleur);
int trouverIndexAlphabet(char c);
void ecrireMot(const char *mot, int xDepart, int yDepart, uint32_t couleur);
int longueurMot(const char *mot);
void defilerTexte(const char *mot, int yDepart, uint32_t couleur);
void ecranAccueil(const char *mot);
void processJson(char *incoming);
bool toutouAttrape(uint32_t temps_actif, int distance_attrape);

void eteindreCompteur();

void eclairage_int_boite(const uint8_t *RGB);
void reinit_LED(int debut, int fin);

void EcranAccueil();
void EcranPerdant();
void EcranGagnant();
void EcranDiff();
void TaskRetroUI(void *pvParameters);

static inline void rxBufferPush(char c);
static inline bool rxBufferPop(char &c);

uint32_t gradient_couleur(float ratio);


void eclairage_LED_ext();

//Global variables for RTOS synchronization
EventGroupHandle_t inputEventGroup;

volatile bool jsonMoveActive = false;

/**
 * @brief Initialise le système embarqué et configure tous les matériels et tâches RTOS.
 *
 * Configure les composants suivants:
 * - Communication série à 115200 baud
 * - Moteur Dynamixel pour la pince (détection d'ID et mode de fonctionnement)
 * - Moteurs pas à pas (axes X, Y et Z)
 * - Boutons de contrôle et leurs interruptions
 * - Fins de course (limit switches)
 * - Bandes de LED (affichage et état)
 * - Tâches FreeRTOS pour le contrôle moteur, la machine d'état, la communication JSON et l'interface
 *
 * @return void
 */
void setup(){
	Serial.begin(115200);
	delay(2000);

    sendFullSnapshot();
    delay(2000);

    //SONAR
    pinMode(43, OUTPUT);
    pinMode(45, INPUT);

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
    dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, 1);
    dxl.writeControlTableItem(ControlTableItem::CURRENT_LIMIT, id, 1);

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

    pixels_LED.begin();
    pixels_LED.clear();
    pixels_LED.show();

	//RTOS
	//----------------
	inputEventGroup = xEventGroupCreate();
	limitEventGroup = xEventGroupCreate();

    serialMutex = xSemaphoreCreateMutex();

	xTaskCreate(TaskMotorControl, "MotorTask", 128, NULL, 3, &hMotorTask);
	xTaskCreate(TaskStateControl, "StateTask", 384, NULL, 4, NULL);
	xTaskCreate(TaskCommJsonSend,    "CommSend",256, NULL, 3, &hCommSend);
	xTaskCreate(TaskCommJsonReceive, "CommRecv", 384, NULL, 3, &hCommRecv);
    xTaskCreate(TaskRetroUI, "RetroUI", 128, NULL, 3, &hRetroUI); //128 is ok, we chose 256 to be safe

    Serial.print(F("Stack headroom CommSend: "));
    Serial.println(uxTaskGetStackHighWaterMark(hCommSend));
    Serial.print(F("Stack headroom MotorTask: "));
    Serial.println(uxTaskGetStackHighWaterMark(hMotorTask));
    Serial.print(F("Stack headroom RetroUI: "));
    Serial.println(uxTaskGetStackHighWaterMark(hRetroUI));

    eclairage_int_boite(ROSE_l);

    vTaskStartScheduler();
}

/**
 * @brief Boucle principale (non utilisée, le système fonctionne avec FreeRTOS).
 *
 * Cette fonction reste vide car la planification est gérée par le planificateur FreeRTOS.
 *
 * @return void
 */
void loop(){
}

/**
 * @brief Effectue le référencement (homing) des axes X et Y en utilisant les fins de course.
 *
 * Positionne le système à l'origine (0,0) en utilisant les capteurs de fin de course.
 * D'abord l'axe X est référencé, puis l'axe Y. Les positions des moteurs sont réinitialisées à 0.
 *
 * @return void
 */
void homeXY(){
    
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
        posX -= 250;
 
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
        posY += 250;
 
        MOT_A.moveTo(posX + posY);
        MOT_B.moveTo(posX - posY);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
 
    MOT_A.setCurrentPosition(0);
    MOT_B.setCurrentPosition(0);
    posY = 0;
}

/**
 * @brief Tâche RTOS qui gère la machine d'état du système et les transitions entre les phases du jeu.
 *
 * Contrôle les états suivants: ACCUEIL, SETUP, DIFF_CHOOSE, IDLE, LOWERING, CLOSING,
 * LIFTING, MOVING_TO_DROPZONE, et DROPPING. Gère les chronomètres, les animations LED
 * et la détection des objets attrapés.
 *
 * @param pvParameters Pointeur sur les paramètres (non utilisé)
 * @return void
 */
void TaskStateControl(void *pvParameters){
  (void) pvParameters;

  for(;;) {

	switch(currentState) {
        case ACCUEIL:
            vTaskSuspend(hCommRecv); // Suspendre la tâche de réception JSON pendant l'écran d'accueil
            vTaskSuspend(hMotorTask); // Suspendre la tâche de contrôle des moteurs pendant l'écran d'accueil
            retroUIState = ECRAN_ACCUEIL;
            xEventGroupWaitBits(inputEventGroup, EVT_BTN_OK, pdTRUE, pdFALSE, portMAX_DELAY);
            currentState = SETUP;
            
            break;
        case SETUP:
            vTaskResume(hCommRecv); // Reprendre la tâche de réception JSON une fois que l'écran d'accueil est passé
            vTaskResume(hMotorTask);
            retroUIState = RIEN;
            xEventGroupWaitBits(inputEventGroup, EVT_BTN_OK, pdTRUE, pdFALSE, portMAX_DELAY);
            //ouvrirPince();
            homeXY();
            //Remonter axe Z à ajouter
			currentState = DIFF_CHOOSE;
			break;
        case DIFF_CHOOSE:
            //vTaskSuspend(hCommRecv); // Suspendre la tâche de réception JSON pendant le choix de la difficulté pour éviter les interférences
            vTaskSuspend(hMotorTask); // Suspendre la tâche de contrôle des moteurs pendant le choix de la difficulté
            retroUIState = SELECTION_DIFFICULTE;
            //vTaskResume(hRetroUI); // S'assurer que la tâche de l'interface utilisateur est active pour afficher le menu de sélection
            
            xEventGroupWaitBits(inputEventGroup, EVT_BTN_OK, pdTRUE, pdFALSE, portMAX_DELAY);

            currentState = IDLE;
            break;
	    case IDLE:{
            retroUIState = TIMER;
            //vTaskResume(hCommRecv); // Reprendre la tâche de réception JSON une fois la difficulté choisie
            vTaskResume(hMotorTask); // Reprendre la tâche de contrôle des moteurs une fois la difficulté choisie
            manualControlEnabled = true; // permettre le controle manuel de l'axe XY avec les boutons
            TickType_t start = xTaskGetTickCount();
            TickType_t duration = pdMS_TO_TICKS(temps[difficulty] * 1000);
            int dernierDecompte = -10;
            static int lastOff = 0; //Pour led strip décompte

            while ((xTaskGetTickCount() - start) < duration) {

                TickType_t elapsedTicks = xTaskGetTickCount() - start;
                int elapsedSec = elapsedTicks / configTICK_RATE_HZ;
                int totalSec = temps[difficulty];

                int decompte = totalSec - elapsedSec;
                if (decompte < 0) decompte = 0;

                if(decompte != dernierDecompte){
                    pixels.clear();
                    char nombreTexte[3];
                    sprintf(nombreTexte, "%02d", decompte);

                    if (decompte<10) {
                        ecrireMot(nombreTexte, 11, 4, pixels.Color(25, 0, 0));
                    }
                    else if(decompte%10 == 0){
                        ecrireMot(nombreTexte, 11, 4, pixels.Color(25, 25, 25));
                    }
                    else {
                        ecrireMot(nombreTexte, 11, 4, pixels.Color(25, 25, 0));
                    }
                    pixels.show();
                    dernierDecompte = decompte;
                }
                const int shouldOff = n_LED_compteur - (int)round(n_LED_compteur*decompte/temps[difficulty]); //nb de lumières qui devraient être éteintes
                if(shouldOff != lastOff){
                    const int shouldOff = n_LED_compteur - (int)round(n_LED_compteur*decompte/temps[difficulty]); //nb de lumières qui devraient être éteintes
                    // Éteindre la différence depuis la dernière fois
                    for (int i = lastOff; i < shouldOff; i++) {
                        int index = n_LED_compteur - 1 - i; // Éteindre de droite à gauche
                        pixels_LED.setPixelColor(OFFSET_LED_COMPTEUR + index, 0);
                    }
                    lastOff = shouldOff;

                    // LEDs restantes
                    const int LED_restant = n_LED_compteur - shouldOff;
                    // Gradient de couleur pour les LEDs restantes
                    float ratio = (float)LED_restant / (float)n_LED_compteur;
                    uint32_t couleur = gradient_couleur(ratio);
                    for (int i = 0; i < LED_restant; i++) {
                        pixels_LED.setPixelColor(OFFSET_LED_COMPTEUR + i, couleur);
                    }
                    pixels_LED.show();
                }

                if (xEventGroupWaitBits(inputEventGroup, EVT_BTN_OK, pdTRUE, pdFALSE, pdMS_TO_TICKS(20))) {
                    break;
                }

                vTaskDelay(pdMS_TO_TICKS(80));
            }

            manualControlEnabled = false; // Disable manual control
            currentState = LOWERING;
			break;
        }
	    case LOWERING:{
			//Séquence de mouvement vers le bas
            long current = MOT_Z.currentPosition();
            long target = maxDownZPos;
            long dir = (target > current) ? 1 : -1;

            while(1){ // Tant que l'axe Z n'est pas presque au plus bas
                MOT_Z.moveTo(MOT_Z.currentPosition() + dir*speed[difficulty]) ; // Update internal position
                if(xEventGroupWaitBits(inputEventGroup, EVT_BTN_OK, pdTRUE, pdFALSE, pdMS_TO_TICKS(20)) || abs(MOT_Z.currentPosition()-maxDownZPos) < 50) {
                    MOT_Z.stop();
                    break; // Sortir de la boucle pour arrêter la descente
                }
                vTaskDelay(pdMS_TO_TICKS(40));
            }
            vTaskSuspend(hMotorTask); // Suspendre la tâche de contrôle des moteurs pendant qu'on ramasse un toutou'
			currentState = CLOSING;
			break;
        }
	    case CLOSING:{  
			fermerPince();
			currentState = LIFTING;
			break;
	    }
	    case LIFTING:{
			//Attendre qu'on soit rendus à liftedZPos, apres retour au home
            vTaskResume(hMotorTask);
			MOT_Z.moveTo(liftedZPos);
            while(abs(MOT_Z.currentPosition() - liftedZPos) > 50) {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
			currentState = MOVING_TO_DROPZONE;
			break;
        }

	    case MOVING_TO_DROPZONE: 
			//Séquence de mouvement vers la zone de dépot
			homeXY();
			currentState = DROPPING;
			break;

	    case DROPPING:{ 
			dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, MOVE_CURRENT);
            dxl.setGoalPosition(id, OPEN_POS, UNIT_RAW); // Ouvrir la pince sans attendre qu'on aie fini d'ouvrir.
            bool win = toutouAttrape(4000,17);
            if(win)retroUIState = ECRAN_GAGNANT;
            else retroUIState = ECRAN_PERDANT;
            vTaskDelay(pdMS_TO_TICKS(5000));
			currentState = DIFF_CHOOSE;
			break;
	    }
	}
	vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/**
 * @brief Tâche RTOS qui envoie périodiquement l'état du système au format JSON via la liaison série.
 *
 * Détecte les changements dans les positions, états, paramètres et états des boutons,
 * puis envoie uniquement les données modifiées au format JSON pour optimiser la bande passante.
 *
 * @param pvParameters Pointeur sur les paramètres (non utilisé)
 * @return void
 */
void TaskCommJsonSend(void *pvParameters){
    (void) pvParameters;

    for (;;) {

        bool send = false;
        doc_o.clear();

        // ---------------- POSITION ----------------
        if (posX != last.posX) {
            doc_o["posX"] = posX;
            last.posX = posX;
            send = true;
        }

        if (posY != last.posY) {
            doc_o["posY"] = posY;
            last.posY = posY;
            send = true;
        }

        long z = MOT_Z.currentPosition();
        if (z != last.zPos) {
            doc_o["zPos"] = z;
            last.zPos = z;
            send = true;
        }

        // ---------------- STATE ----------------
        if ((int)currentState != last.state) {
            doc_o["state"] = (int)currentState;
            last.state = (int)currentState;
            send = true;
        }

        if (ACTUAL_POS != last.ACTUAL_POS) {
            doc_o["pos_act"] = ACTUAL_POS;
            last.ACTUAL_POS = ACTUAL_POS;
            send = true;
        }

        if (difficulty != last.difficulty) {
            doc_o["diff"] = difficulty;
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
            JsonArray arrTemps = doc_o.createNestedArray("temps");
            for (int i = 0; i < 3; i++) {
                arrTemps.add(temps[i]);
                last.temps[i] = temps[i];
            }
            send = true;
        }
        
        if (forceChanged) {
            JsonArray arrForce = doc_o.createNestedArray("force");
            for (int i = 0; i < 3; i++) {
                arrForce.add(force[i]);

                last.force[i] = force[i];

            }

            send = true;

        }
     
        if (speedChanged) {
            JsonArray arrSpeed = doc_o.createNestedArray("speed");
            for (int i = 0; i < 3; i++) {
                arrSpeed.add(speed[i]);
                last.speed[i] = speed[i];
            }
            send = true;
        }
        
        // ---------------- PINCE ----------------
        if (OPEN_POS != last.OPEN_POS) {
            doc_o["pince_open"] = OPEN_POS;
            last.OPEN_POS = OPEN_POS;
            send = true;
        }

        if (CLOSED_POS != last.CLOSED_POS) {
            doc_o["pince_closed"] = CLOSED_POS;
            last.CLOSED_POS = CLOSED_POS;
            send = true;
        }

        // ---------------- LIMITS ----------------
        if (MAX_POS_X != last.MAX_POS_X) {
            doc_o["max_x"] = MAX_POS_X;
            last.MAX_POS_X = MAX_POS_X;
            send = true;
        }

        if (MAX_POS_Y != last.MAX_POS_Y) {
            doc_o["max_y"] = MAX_POS_Y;
            last.MAX_POS_Y = MAX_POS_Y;
            send = true;
        }

        if (liftedZPos != last.liftedZPos) {
            doc_o["z_high"] = liftedZPos;
            last.liftedZPos = liftedZPos;
            send = true;
        }

        if (maxDownZPos != last.maxDownZPos) {
            doc_o["z_down"] = maxDownZPos;
            last.maxDownZPos = maxDownZPos;
            send = true;
        }

        // ---------------- LED ----------------
        if (ledColor != last.ledColor) {
            doc_o["led"] = ledColor;
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
            doc_o["btn_up"] = up;
            last.btnUp = up;
            send = true;
        }

        if (down != last.btnDown) {
            doc_o["btn_down"] = down;
            last.btnDown = down;
            send = true;
        }

        if (left != last.btnLeft) {
            doc_o["btn_left"] = left;
            last.btnLeft = left;
            send = true;
        }

        if (right != last.btnRight) {
            doc_o["btn_right"] = right;
            last.btnRight = right;
            send = true;
        }

        if (ok != last.btnOk) {
            doc_o["btn_ok"] = ok;
            last.btnOk = ok;
            send = true;
        }

        // ---------------- SEND ONLY IF CHANGED ----------------
        if (send) {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                serializeJson(doc_o, Serial);
                Serial.print('\n');
                xSemaphoreGive(serialMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief Envoie un instantané complet de tous les paramètres du système au format JSON.
 *
 * Utilisée à l'initialisation pour synchroniser l'interface distante avec l'état actuel du système.
 * Inclut les positions, états, paramètres de la pince, limites de déplacement et couleurs LED.
 *
 * @return void
 */
void sendFullSnapshot(){
    static StaticJsonDocument<768> doc1;

    doc1["type"] = "full";

    doc1["posX"] = posX;
    doc1["posY"] = posY;
    doc1["zPos"] = MOT_Z.currentPosition();
    doc1["state"] = (int)currentState;
    doc1["diff"] = difficulty;

    // --- Pince ---
    JsonObject pince = doc1.createNestedObject("pince");
    pince["pos_o"] = OPEN_POS;
    pince["pos_f"] = CLOSED_POS;
    pince["pos_act"] = ACTUAL_POS;

    // --- Limits ---
    JsonObject limits = doc1.createNestedObject("limits");
    limits["maxPosX"] = MAX_POS_X;
    limits["maxPosY"] = MAX_POS_Y;
    limits["maxH"] = liftedZPos;
    limits["minH"] = maxDownZPos;

    // --- Arrays ---
    doc1["temps"][0] = temps[0];
    doc1["temps"][1] = temps[1];
    doc1["temps"][2] = temps[2];

    doc1["force"][0] = force[0];
    doc1["force"][1] = force[1];
    doc1["force"][2] = force[2];

    doc1["speed"][0] = speed[0];
    doc1["speed"][1] = speed[1];
    doc1["speed"][2] = speed[2];

    //doc1["ledColor"] = ledColor;
    doc1["led"] = ledColor;

    // --- Buttons ---
    JsonObject buttons = doc1.createNestedObject("buttons");
    buttons["haut"]   = btnUp;
    buttons["bas"]    = btnDown;
    buttons["gauche"] = btnLeft;
    buttons["droite"] = btnRight;
    buttons["ok"]     = false;

    serializeJson(doc1, Serial);
    Serial.print('\n');
}

/**
 * @brief Tâche RTOS qui contrôle les moteurs pas à pas (XY et Z).
 *
 * Lit les états des boutons pour le contrôle manuel, calcule les positions cibles en coordonnées XY,
 * puis exécute les mouvements des moteurs selon les instructions de la machine d'état ou des entrées JSON.
 *
 * @param pvParameters Pointeur sur les paramètres (non utilisé)
 * @return void
 */
void TaskMotorControl(void *pvParameters){
    (void) pvParameters;

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

                if (up)    targetX += speed[difficulty];
                if (down)  targetX -= speed[difficulty];
                if (left)  targetY += speed[difficulty]; 
                if (right) targetY -= speed[difficulty]; 

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

/**
 * @brief Routine d'interruption déclenchée par les fins de course (limit switches).
 *
 * Détecte quand les axes X ou Y atteignent leurs limites et signale l'événement via un groupe d'événements.
 * Utilise un déclenchement sur flanc descendant (FALLING).
 *
 * @return void
 */
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

/**
 * @brief Routine d'interruption du bouton OK avec gestion de l'anti-rebond.
 *
 * Détecte les appuis sur le bouton OK en ignorant les rebonds pendant la période de debounce (20 ms).
 * Signale l'événement via le groupe d'événements.
 *
 * @return void
 */
void NotifyOkButton(){
    TickType_t now = xTaskGetTickCountFromISR();
    if ((now - lastBtnInterrupt < pdMS_TO_TICKS(DEBOUNCE_MS)) || digitalRead(BTN_PIN_OK) == HIGH) return;
    lastBtnInterrupt = now;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (digitalRead(BTN_PIN_OK) == LOW)
    xEventGroupSetBitsFromISR(inputEventGroup, EVT_BTN_OK, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

/**
 * @brief Ouvre complètement la pince Dynamixel.
 *
 * Positionne la pince à OPEN_POS avec le courant MOVE_CURRENT et attend que le moteur atteigne la position.
 * Boucle jusqu'à ce que la position actuelle soit à moins de 50 unités de OPEN_POS.
 *
 * @return void
 */
void ouvrirPince(){
    dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, MOVE_CURRENT);
    dxl.setGoalPosition(id, OPEN_POS, UNIT_RAW);
    while (ACTUAL_POS > OPEN_POS + 50) { // Tant que la pince n'est pas presque ouverte
        vTaskDelay(pdMS_TO_TICKS(50));
        ACTUAL_POS = dxl.getPresentPosition(id);
    }
}

/**
 * @brief Ferme la pince Dynamixel avec une force dépendante du niveau de difficulté.
 *
 * Positionne la pince à CLOSED_POS avec le courant défini par force[difficulty].
 * Boucle jusqu'à ce que la position actuelle soit à moins de 50 unités de CLOSED_POS.
 *
 * @return void
 */
void fermerPince(){
    dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, force[difficulty]);
    dxl.setGoalPosition(id, CLOSED_POS, UNIT_RAW);
    while (ACTUAL_POS < CLOSED_POS - 50) { // Tant que la pince n'est pas presque fermée
        vTaskDelay(pdMS_TO_TICKS(50));
        ACTUAL_POS = dxl.getPresentPosition(id);
    }
}

/**
 * @brief Analyse et traite les messages JSON reçus depuis l'interface distante.
 *
 * Supporte trois types de messages: "commande" (mouvements et actions), "pers" (paramètres personnalisés),
 * et "rep" (réglages de configuration). Exécute les actions appropriées selon le type et le contenu du message.
 *
 * @param incoming Chaîne de caractères contenant le message JSON à traiter
 * @return void
 */
void processJson(char *incoming){
    doc_i.clear();
    DeserializationError err = deserializeJson(doc_i, incoming);
    
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
 
    const char* type = doc_i["type"];
    if (!type) {
        return;
    }
 
    if (strcmp(type, "commande") == 0) {
        const char* action = doc_i["action"];
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
        if (doc_i["clr"].is<int>()) {
            ledColor = (LedColor)doc_i["clr"].as<int>();
        }
 
        if (doc_i["diff"].is<const char*>()) {
            const char* diff = doc_i["diff"];
 
            if (strcmp(diff, "fac") == 0) difficulty = 0;
            else if (strcmp(diff, "moy") == 0) difficulty = 1;
            else if (strcmp(diff, "exp") == 0) difficulty = 2;
 
            temps[difficulty] = doc_i["t"] | temps[difficulty];
            force[difficulty] = doc_i["F"] | force[difficulty];
            speed[difficulty] = doc_i["v"] | speed[difficulty];
        }
 
        return;
    }
 
    if (strcmp(type, "rep") == 0) {
        const char* champ = doc_i["champ"];
        int32_t valeur = doc_i["valeur"] | 0;
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

/**
 * @brief Ajoute un caractère au tampon de réception circulaire.
 *
 * Si le tampon est plein, écrase le caractère le plus ancien. Utilisée par l'interruption de réception série.
 *
 * @param c Caractère à ajouter au tampon
 * @return void
 */
static inline void rxBufferPush(char c){
    uint16_t next = (rxHead + 1) % RX_BUF_SIZE;

    if (next == rxTail) {
        rxTail = (rxTail + 1) % RX_BUF_SIZE;
    }
    rxBuf[rxHead] = c;
    rxHead = next;
}

/**
 * @brief Récupère et enlève le caractère suivant du tampon de réception circulaire.
 *
 * @param c Référence au caractère qui recevra la valeur extraite du tampon
 * @return true si un caractère a été extrait, false si le tampon est vide
 */
static inline bool rxBufferPop(char &c){
    if (rxTail == rxHead) return false;

    c = rxBuf[rxTail];
    rxTail = (rxTail + 1) % RX_BUF_SIZE;
    return true;
}

/**
 * @brief Tâche RTOS qui reçoit et traite les messages JSON depuis la liaison série.
 *
 * Lit les données du port série, les accumule dans un tampon circulaire, reconstruit les messages
 * ligne par ligne (séparés par des retours à la ligne), puis appelle processJson pour chaque message complet.
 *
 * @param pvParameters Pointeur sur les paramètres (non utilisé)
 * @return void
 */
void TaskCommJsonReceive(void *pvParameters){
    (void) pvParameters;

    static char lineBuf[128]; //128 tto FOURCHETTE
    static uint16_t lineLen = 0;

    for (;;) {

        while (Serial.available()) {
            char c = Serial.read();
            rxBufferPush(c);
        }
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

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

//---------------------
// SONAR
//----------------------

/**
 * @brief Détecte si un objet a été attrapé en utilisant le capteur ultrason (sonar).
 *
 * Mesure la distance de l'objet pendant une période donnée et détecte les changements de distance.
 * Retourne vrai si la distance s'éloigne de plus de 2 cm de la distance cible initiale.
 *
 * @param temps_actif Durée en millisecondes pendant laquelle détecter la capture
 * @param distance_attrape Distance de référence en centimètres de l'objet attendu
 * @return true si un objet a été attrapé, false sinon
 */
bool toutouAttrape(uint32_t temps_actif, int distance_attrape){
    float temps_retour_signal;
    float distance_parcourue;
    int reussi = 0;

    uint32_t start = millis();

    while ((millis() - start) < temps_actif) {

        digitalWrite(43, HIGH);
        delayMicroseconds(10); // OK
        digitalWrite(43, LOW);

        temps_retour_signal = pulseIn(45, HIGH, 30000); // réduire timeout

        distance_parcourue = (temps_retour_signal * 0.0343) / 2;
        //Serial.println(distance_parcourue);

        if (distance_parcourue > 0.1) {
            if (abs(distance_parcourue-distance_attrape) > 2 && reussi == 0) {
                reussi = 1;
            }
            else if (abs(distance_parcourue-distance_attrape) < 2 && reussi == 1) {
                reussi = 0;
            }
            else if (abs(distance_parcourue-distance_attrape) > 2 && reussi == 1) {
                return true;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return false;
}

//----------------------
// UI RETRO
//----------------------

/**
 * @brief Transforme les coordonnées d'une demi-matrice (14x13) en index LED.
 *
 * Convertit les coordonnées x, y en index linéaire pour la bibliothèque NeoPixel.
 *
 * @param x Coordonnée X (0-13)
 * @param y Coordonnée Y (0-12)
 * @return Index LED dans le tableau de pixels (0-181)
 */
int transformationIntermediaire(int x, int y){
    int indexIntermediaire = (y * largeur) + x;
    return indexIntermediaire;
};
 
/**
 * @brief Transforme les coordonnées de la matrice LED complète (28x13) en index LED.
 *
 * Gère deux demi-matrices côte à côte en appelant transformationIntermediaire pour chacune.
 *
 * @param x Coordonnée X (0-27)
 * @param y Coordonnée Y (0-12)
 * @return Index LED dans le tableau complet de pixels (0-363)
 */
int transformationCoordonnees(int x, int y){
    int index = 0;
    if (x < largeur){
        index = transformationIntermediaire(x, y);
    }
    else{
        int xLocal = x - largeur;
        index = transformationIntermediaire(xLocal, y) + (largeur * hauteur);
    }
    
    return index;
};
 
/**
 * @brief Allume une LED aux coordonnées spécifiées avec la couleur donnée.
 *
 * Vérifie que les coordonnées sont dans les limites de la matrice (0-27, 0-12)
 * puis définit la couleur du pixel correspondant.
 *
 * @param x Coordonnée X de la LED (0-27)
 * @param y Coordonnée Y de la LED (0-12)
 * @param couleur Couleur RGB au format pixels.Color(R,G,B)
 * @return void
 */
void allumeLED(int x, int y, uint32_t couleur){
    if (x >= 0 && x < 28 && y >= 0 && y < 13){
        int index = transformationCoordonnees(x, y);
        pixels.setPixelColor(index, couleur);
    }
};
 
/**
 * @brief Affiche une lettre sur la matrice LED à partir d'une matrice de pixels 5x3.
 *
 * Lit la matrice de caractères stockée en mémoire PROGMEM et allume les LEDs correspondantes.
 *
 * @param matriceLettre Matrice 5x3 représentant le motif de la lettre
 * @param xDepart Coordonnée X de départ pour afficher la lettre
 * @param yDepart Coordonnée Y de départ pour afficher la lettre
 * @param couleur Couleur RGB pour afficher la lettre
 * @return void
 */
void ecrireLettre(const uint8_t matriceLettre[5][3], int xDepart, int yDepart, uint32_t couleur){
    for (int rangee = 0; rangee < 5; rangee++){
        for (int colonne = 0; colonne < 3; colonne++){
            if (pgm_read_byte(&matriceLettre[rangee][colonne]) == 1){
                allumeLED(xDepart + colonne, yDepart + rangee, couleur);
            }
        }
    }
};
 
/**
 * @brief Trouve l'index d'un caractère dans le tableau d'alphabet défini.
 *
 * Supporte les lettres majuscules (A-Z), les chiffres (0-9) et quelques caractères spéciaux (!, :, /).
 *
 * @param c Caractère à rechercher
 * @return Index dans le tableau alphabet, ou -1 si le caractère n'est pas trouvé
 */
int trouverIndexAlphabet(char c){
    int index = 0;
    if (c >= 'A' && c <= 'Z'){
        index = c - 'A';
    }
    else if (c >= '0' && c <= '9'){
        index = 26 + c - '0';
    }
    else if (c == '!'){
        index = 36;
    }
    else if (c == ':'){
        index = 37;
    }
    else if (c == '/'){
        index = 38;
    }
    else{
        index = -1;
    }
    return index;
};
 
/**
 * @brief Affiche un mot complet sur la matrice LED.
 *
 * Affiche chaque caractère du mot en l'espaçant de largeurLettre pixels horizontalement.
 *
 * @param mot Chaîne de caractères à afficher
 * @param xDepart Coordonnée X de départ pour afficher le mot
 * @param yDepart Coordonnée Y de départ pour afficher le mot
 * @param couleur Couleur RGB pour afficher le mot
 * @return void
 */
void ecrireMot(const char *mot, int xDepart, int yDepart, uint32_t couleur){
    for (int i = 0; mot[i] != '\0'; i++){
        char lettre = mot[i];
        int index = trouverIndexAlphabet(lettre);
        if (index != -1){
            ecrireLettre(alphabet[index], (xDepart + i * largeurLettre), yDepart, couleur);
        }
    }
}
 
/**
 * @brief Calcule la longueur d'une chaîne de caractères.
 *
 * Compte le nombre de caractères jusqu'au terminateur nul.
 *
 * @param mot Chaîne de caractères dont on veut connaître la longueur
 * @return Nombre de caractères (sans le terminateur nul)
 */
int longueurMot(const char *mot){
    int longueur = 0;
    while (mot[longueur] != '\0'){
        longueur++;
    }
    return longueur;
}

/**
 * @brief Fait défiler horizontalement un texte sur la matrice LED de droite à gauche.
 *
 * Le texte commence à la droite de l'écran et disparaît à la gauche.
 * À chaque appel, le texte avance d'un pixel vers la gauche.
 *
 * @param mot Texte à faire défiler
 * @param yDepart Coordonnée Y du texte à défiler
 * @param couleur Couleur RGB du texte
 * @return void
 */
void defilerTexte(const char *mot, int yDepart, uint32_t couleur){
    static int x = 28;
    int largeurMot = longueurMot(mot) * 4;
    ecrireMot(mot, x, yDepart, couleur);
    x--;
    if (x < -largeurMot){
        x = 28;
    }
}

/**
 * @brief Tâche RTOS qui gère l'affichage rétro de la matrice LED.
 *
 * Met à jour l'affichage selon l'état actuel (menu d'accueil, sélection de difficulté, écrans gagnant/perdant).
 * Gère également l'éclairage interne et externe de la machine.
 *
 * @param pvParameters Pointeur sur les paramètres (non utilisé)
 * @return void
 */
void TaskRetroUI(void *pvParameters){
    (void) pvParameters;
    LedColor oldColor = -1;
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
            case TIMER:
                break;
            case RIEN:
                [[fallthrough]];
            default:
                pixels.clear();
                pixels.show();
                break;
        };
        if(oldColor != ledColor){
            switch(ledColor){
                case(LED_ROUGE):
                    eclairage_int_boite(ROUGE_l);
                    break;
                case(LED_ORANGE):
                    eclairage_int_boite(ORANGE_l);
                    break;
                case(LED_JAUNE):
                    eclairage_int_boite(JAUNE_l);
                    break;
                case(LED_VERT):
                    eclairage_int_boite(VERT_l);
                    break;
                case(LED_BLEU):
                    eclairage_int_boite(BLEU_l);
                    break;
                case(LED_MAUVE):
                    eclairage_int_boite(MAUVE_l);
                    break;
                case(LED_ROSE):
                    eclairage_int_boite(ROSE_l);
                    break;
                case(LED_BLANC):
                    eclairage_int_boite(BLANC_l);
                    break;
            }
            oldColor = ledColor;
        }
        eclairage_LED_ext();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/**
 * @brief Affiche l'écran d'accueil avec le texte "GROCHET" et des bandes de couleurs animées.
 *
 * Les bandes de couleur défilent horizontalement à chaque appel pour créer un effet d'animation.
 *
 * @return void
 */
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
    decalage++;
    if (decalage == 3) decalage = 0;
}

/**
 * @brief Affiche l'écran de défaite avec le texte "MEILLEURE CHANCE LA PROCHAINE FOIS" défilant.
 *
 * Le texte bleu défile de droite à gauche avec un motif de petits "x" rouges en haut et en bas de l'écran.
 *
 * @return void
 */
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

/**
 * @brief Affiche l'écran de victoire avec le texte "Bravo!" et des motifs de cœurs roses.
 *
 * L'écran clignote en alternant l'affichage du texte et des cœurs.
 *
 * @return void
 */
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
    if (afficher) ecrireMot("Bravo!", 3, 4, pixels.Color(0, 20, 5));

    afficher = !afficher;
    pixels.show();
}

/**
 * @brief Affiche l'écran de sélection de la difficulté du jeu.
 *
 * Affiche le texte "CHOIX DIFFICULTE" qui défile, la difficulté actuelle (FACILE, MOYEN, EXPERT)
 * et une flèche directionnelle. Les boutons gauche/droite permettent de changer la difficulté.
 *
 * @return void
 */
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

//----------------------------
// LED Strips
//----------------------------

/**
 * @brief Éteint les LEDs dans la plage spécifiée.
 *
 * Réinitialise la couleur de toutes les LEDs de la plage à noir (éteint).
 *
 * @param debut Index de début de la plage de LEDs à éteindre (inclus)
 * @param fin Index de fin de la plage de LEDs à éteindre (exclu)
 * @return void
 */
void reinit_LED(int debut, int fin){
    for(int i = debut; i<fin; i++){
        pixels_LED.setPixelColor(i, pixels_LED.Color(0,0,0));
    }
}

/**
 * @brief Contrôle l'éclairage interne de la machine.
 *
 * Éteint d'abord toutes les LEDs internes, puis les allume avec la couleur RGB spécifiée.
 *
 * @param RGB Pointeur sur un tableau de 3 octets [R, G, B] (valeurs 0-255)
 * @return void
 */
void eclairage_int_boite(const uint8_t *RGB){ 
    reinit_LED(OFFSET_LED_INT, n_LED_total);
    for(int i = OFFSET_LED_INT; i < n_LED_total; i++){
        pixels_LED.setPixelColor(i, pixels_LED.Color(RGB[0], RGB[1], RGB[2]));
    }
    
    pixels_LED.show();
};

/**
 * @brief Anime l'éclairage externe (bandes LED verticales) de la machine.
 *
 * Affiche un motif rotatif en trois couleurs (rose, orange, bleu) qui se décale à chaque appel,
 * créant l'impression d'un mouvement continu des bandes LED avant et arrière.
 *
 * @return void
 */
void eclairage_LED_ext(){

    static int decalage = 0;
    int longueur = NUMPIXELS_BANDE * 2;

    //Réinitialisation des LEDs extérieures
    reinit_LED(OFFSET_LED_GAUCHE, OFFSET_LED_COMPTEUR -1);
    reinit_LED(OFFSET_LED_DROIT, OFFSET_LED_INT -1);

    for (int i = 0; i < longueur; i++)
    {
        int couleur = (i + decalage) % 3;
        uint32_t color;

        if(couleur == 0){
            color = pixels_LED.Color(ROSE_l[0], ROSE_l[1], ROSE_l[2]);
        }
        else if(couleur == 1){
            color = pixels_LED.Color(ORANGE_l[0], ORANGE_l[1], ORANGE_l[2]);
        }
        else{
            color = pixels_LED.Color(BLEU_l[0], BLEU_l[1], BLEU_l[2]);
        }
        pixels_LED.setPixelColor(OFFSET_LED_GAUCHE + i, color);  
        pixels_LED.setPixelColor(OFFSET_LED_DROIT + i, color);
    }
    pixels_LED.show();
    decalage = (decalage + 1) % 3;
};

/**
 * @brief Éteint toutes les LEDs du compteur de temps.
 *
 * Remet les LEDs du compteur (OFFSET_LED_COMPTEUR) à noir.
 *
 * @return void
 */
void eteindreCompteur(){
    for (int i = 0; i < n_LED_compteur; i++) {

      pixels_LED.setPixelColor(OFFSET_LED_COMPTEUR + i, 0);
    }
    pixels_LED.show();
}

/**
 * @brief Allume toutes les LEDs du compteur de temps en vert.
 *
 * Éclaire le compteur avec la couleur verte (VERT_l) pour indiquer le temps restant disponible.
 *
 * @return void
 */
void allumerCompteur(){
    for (int i = 0; i < n_LED_compteur; i++) {
      pixels_LED.setPixelColor(OFFSET_LED_COMPTEUR + i, pixels_LED.Color(VERT_l[0], VERT_l[1], VERT_l[2]));
    }
    pixels_LED.show();
}

/**
 * @brief Calcule une couleur de gradient du vert au rouge en passant par le jaune.
 *
 * Pour un ratio de 1.0 (100%), la couleur est verte. À 0.0 (0%), la couleur est rouge.
 * La transition passe par le jaune à ratio = 0.5. Utilé pour le compteur de temps.
 *
 * @param ratio Rapport entre 0.0 et 1.0 indiquant la position dans le dégradé
 * @return Couleur RGB au format pixels_LED.Color(R, G, B)
 */
uint32_t gradient_couleur(float ratio){

  ratio = constrain(ratio, 0.0f, 1.0f);
  float r, g;

  //Transforme le vert en jaune
  if(ratio > 0.5f){
    float ratio_jaune = (1.0f - ratio) / 0.5f;
    r = VERT_l[0] + ratio_jaune * (JAUNE_l[0] - VERT_l[0]);
    g = VERT_l[1] + ratio_jaune * (JAUNE_l[1] - VERT_l[1]);

  }
  //Transforme le jaune en rouge
  else{
    float ratio_rouge = (0.5f - ratio) / 0.5f;
    r = JAUNE_l[0] + ratio_rouge * (ROUGE_l[0] - JAUNE_l[0]);
    g = JAUNE_l[1] + ratio_rouge * (ROUGE_l[1] - JAUNE_l[1]);

  }
  return pixels_LED.Color(int(r), int(g), 0);

}

//TODO
//test limites XY et switch

//Test force pince

//Test latence

//enlever les suspend task