#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "event_groups.h"
#include <AccelStepper.h>

/*Code pour tester le branchement des boutons, teste effectuer le 2026-03-11 par Cloé
1. Commencer par tester la lecture du bouton rouge. Résulta : 0 -> appuyer et 1 -> pas appuyer
2. Tester la lecture de tous les bouton. Résulta : 

J'ai inverser les numéro de pin de Left et right pour corespondre couleur

Rouge : 0 -> appuyer et 1 -> pas appuyer
Bleu : 0 -> appuyer et 0 -> pas appuyer ----> Pas fonctionner (pin 25)
Jaune : 0 -> appuyer et 1 -> pas appuyer
Vert : 0 -> appuyer et 0 -> pas appuyer ----> Pas fonctionner (pin 23)
Blanc : 0 -> appuyer et 1 -> pas appuyer

3. Test de la logique de lorsque bouton = low signifie actionner -> Fonctionnel
*/

//BOUTONS
#define BTN_PIN_UP 22
#define BTN_PIN_DOWN 25
#define BTN_PIN_LEFT 24
#define BTN_PIN_RIGHT 23
#define BTN_PIN_OK 26
#define DEBOUNCE_MS 20

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(BTN_PIN_UP, INPUT_PULLUP); //Verifier avec Marcob input vs input_pullup (resistance interne ou pas)
  pinMode(BTN_PIN_DOWN, INPUT_PULLUP);
  pinMode(BTN_PIN_LEFT, INPUT_PULLUP);
  pinMode(BTN_PIN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_PIN_OK, INPUT_PULLUP);
}

void loop() {  
  // 1.
  Serial.print(digitalRead(BTN_PIN_UP));
  Serial.println("  Rouge");
  
  // 2.
  Serial.print(digitalRead(BTN_PIN_DOWN));
  Serial.println("  Bleu");

  Serial.print(digitalRead(BTN_PIN_LEFT));
  Serial.println("  Jaune");

  Serial.print(digitalRead(BTN_PIN_RIGHT));
  Serial.println("  Vert");

  Serial.print(digitalRead(BTN_PIN_OK));
  Serial.println("  Blanc");

  // 3.
  if(digitalRead(BTN_PIN_UP) == LOW) {
    Serial.println("Bouton Rouge appuyer");
  }
  Serial.println("------------");

  delay(2000);
}