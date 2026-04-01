#include <Arduino.h>
#include <AccelStepper.h>

//MOTEURS
#define FULLSTEP 4 // 4 fils par moteurs
#define STEPS_REV 4096

//Moteur Z
#define EN_PIN_MZ 35
#define DIR_PIN_MZ 33
#define STEP_PIN_MZ 34

#define SerialGripper    Serial2


AccelStepper MOT_Z(AccelStepper::DRIVER, STEP_PIN_MZ, DIR_PIN_MZ);

long stepIncrement = 100; // Number of steps per key press

void setup() {
  Serial.begin(115200);
  SerialGripper.begin(9600);
  delay(2000);

  // Moteur Z
  pinMode(EN_PIN_MZ, OUTPUT);
  digitalWrite(EN_PIN_MZ, LOW); // Enable motor

  MOT_Z.setMaxSpeed(4000);
  MOT_Z.setAcceleration(1000);

  Serial.println("Control Z-axis with UP/DOWN arrows sent via Serial");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    // Check which key was pressed
    if (c == 'w') {          // Up arrow alternative
      MOT_Z.moveTo(MOT_Z.currentPosition() + stepIncrement);
    } else if (c == 's') {   // Down arrow alternative
      MOT_Z.moveTo(MOT_Z.currentPosition() - stepIncrement);
    }
    else if (c == 'x') {
        fermerPince();
    }
    else if (c == ' '){
        ouvrirPince();
    }
  }

  // Keep motor running towards target
  MOT_Z.run();
}