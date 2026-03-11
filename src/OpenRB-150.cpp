#include <Dynamixel2Arduino.h>
#include <FreeRTOS_SAMD21.h>

#define DXL_SERIAL      Serial1
#define DXL_DIR_PIN     -1

int32_t OPEN_POS = 1083
int32_t CLOSED_POS = 9000

#define MOVE_CURRENT    300 // Courant à appliquer pour déplacer la pince (unités brutes)
#define GRIP_CURRENT    50 // Courant à appliquer pour fermer la pince (unités brutes)
#define STALL_CURRENT   40 // Courant en dessous duquel on considère que le moteur est en stall (unités brutes)
#define VEL_THRESHOLD   5 // Vitesse en dessous de laquelle on considère que le moteur est à l'arrêt (unités brutes)
#define STALL_CONFIRM   300 // Temps de suite que le moteur doit être à l'arrêt pour confirmer un stall (ms)

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
uint8_t  id          = 102;

void ouvrirPince();
void fermerPince();
int  detecterToutou();
void calibrer();

void setup() {
    Serial.begin(115200);
    Serial3.begin(115200);  //Peut etre 1 ? Pour communication UART
    delay(2000);

    dxl.begin(57600);
    dxl.setPortProtocolVersion(2.0);

    for (uint8_t i = 1; i < 253; i++) {
        if (dxl.ping(i)) {
            Serial.print("Found Dynamixel ID: ");
            Serial.println(i);
            id = i;
        }
    }

    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_EXTENDED_POSITION);
    dxl.torqueOn(id);

    dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, MOVE_CURRENT);
    Serial.println("Ready.");
}

void loop() {
    // ── Read message from Arduino ──
    if (!Serial3.available()) return;

    uint8_t msg = Serial3.read();
    uint8_t response = 0;

    // ── Act on bits ──
    if (msg & 0x01) ouvrirPince();
    if (msg & 0x02) fermerPince();
    if (msg & 0x08) {
        calibrer();
        response |= 0x04; // Calibration done
    }

    if (msg & 0x04) { //4 = bit qui dit de verifier toutou
        int status = detecterToutou();
        if (status == 1) response |= 0x01;  // toutou_attrape
        if (status == 2) {
            ouvrirPince(); // éviter de staller le moteur
            response |= 0x02;  // rien attrapé
        }
        if (status == 0) response |= 0x04;  // en mouvement (ni toutou attrapé ni rien attrapé)
    }

    // ── Send response ──
    Serial3.write(response);
}

void ouvrirPince() {
    dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, MOVE_CURRENT);
    dxl.setGoalPosition(id, OPEN_POS, UNIT_RAW);
}

void fermerPince() {
    dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, GRIP_CURRENT);
    dxl.setGoalPosition(id, CLOSED_POS, UNIT_RAW);
}

void calibrer() {
    Serial.println("=== CALIBRATION ===");
    dxl.torqueOff(id);  // let user move freely

    // ── Open position ──
    Serial.println("Move gripper to OPEN position, then wait...");
    delay(5000);
    OPEN_POS = dxl.getPresentPosition(id);
    Serial.print("OPEN_POS locked: "); Serial.println(OPEN_POS);

    // ── Closed position ──
    Serial.println("Move gripper to CLOSED position, then wait...");
    delay(5000);
    CLOSED_POS = dxl.getPresentPosition(id);
    Serial.print("CLOSED_POS locked: "); Serial.println(CLOSED_POS);

    dxl.torqueOn(id);
    Serial.print("Range: "); Serial.println(CLOSED_POS - OPEN_POS);
    Serial.println("Calibration done.");
}

int detecterToutou() {
    int32_t pos     = dxl.getPresentPosition(id);
    int32_t vel     = dxl.getPresentVelocity(id);
    int16_t current = (int16_t)dxl.readControlTableItem(ControlTableItem::PRESENT_CURRENT, id);

    Serial.print("Pos: "); Serial.print(pos);
    Serial.print(" | Vel: "); Serial.print(vel);
    Serial.print(" | Current: "); Serial.println(current);

    //Pas de toutou attrapé
    if (pos >= CLOSED_POS - 5) return 2;

    //Toutou attrapé
    if (abs(current) >= STALL_CURRENT && abs(vel) < VEL_THRESHOLD) {
        delay(STALL_CONFIRM); //problématique = delai
        current = (int16_t)dxl.readControlTableItem(ControlTableItem::PRESENT_CURRENT, id);
        vel     = dxl.getPresentVelocity(id);
        if (abs(current) >= STALL_CURRENT && abs(vel) < VEL_THRESHOLD) return 1;
    }

    return 0;
}