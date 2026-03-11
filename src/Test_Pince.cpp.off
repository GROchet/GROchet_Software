#include <Dynamixel2Arduino.h>

#define DXL_SERIAL      Serial1
#define DXL_DIR_PIN     -1

int32_t OPEN_POS = 1083;
int32_t CLOSED_POS = 9000;

#define MOVE_CURRENT    300
#define GRIP_CURRENT    50
#define STALL_CURRENT   40
#define VEL_THRESHOLD   5
#define STALL_CONFIRM   300

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
uint8_t  id          = 102;

void ouvrirPince();
void fermerPince();
int  detecterToutou();
void calibrer();
void waitForSpacebar();

void setup() {
    Serial.begin(115200);
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

    calibrer();
}

void loop() {
    ouvrirPince();
    Serial.println("Pince ouverte");
    delay(2000);

    fermerPince();
    Serial.println("Fermeture en cours...");
    delay(300);

    int status = 0;
    while (status == 0) {
        status = detecterToutou();
        delay(100);
    }

    if (status == 1) {
        Serial.println("Toutou attrapé !");
        delay(3000);
    } else if (status == 2) {
        Serial.println("Rien attrapé.");
        ouvrirPince();
        delay(1000);
    }
}

void waitForSpacebar() {
    Serial.println("  → Press SPACEBAR to confirm...");
    while (true) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == ' ') return;
        }
    }
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
    dxl.torqueOff(id);

    // ── Open position ──
    Serial.println("Move gripper to OPEN position.");
    waitForSpacebar();
    OPEN_POS = dxl.getPresentPosition(id);
    Serial.print("OPEN_POS locked: "); Serial.println(OPEN_POS);

    // ── Closed position ──
    Serial.println("Move gripper to CLOSED position.");
    waitForSpacebar();
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

    if (pos >= CLOSED_POS - 5) return 2;

    if (abs(current) >= STALL_CURRENT && abs(vel) < VEL_THRESHOLD) {
        delay(STALL_CONFIRM);
        current = (int16_t)dxl.readControlTableItem(ControlTableItem::PRESENT_CURRENT, id);
        vel     = dxl.getPresentVelocity(id);
        if (abs(current) >= STALL_CURRENT && abs(vel) < VEL_THRESHOLD) return 1;
    }

    return 0;
}