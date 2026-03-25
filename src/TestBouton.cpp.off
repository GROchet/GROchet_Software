#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "event_groups.h"
#include <AccelStepper.h>

/*Code pour tester le branchement des boutons
1. Commencer par tester la lecture du bouton rouge. Résulta : 0 -> appuyer et 1 -> pas appuyer
2. Tester la lecture de tous les bouton. Résulta : 

(2026-03-11) Cloé
J'ai inverser les numéro de pin de Left et right pour corespondre couleur

Rouge : 0 -> appuyer et 1 -> pas appuyer
Bleu : 0 -> appuyer et 0 -> pas appuyer ----> Pas fonctionner (pin 25)
Jaune : 0 -> appuyer et 1 -> pas appuyer
Vert : 0 -> appuyer et 0 -> pas appuyer ----> Pas fonctionner (pin 23)
Blanc : 0 -> appuyer et 1 -> pas appuyer

3. Test de la logique de lorsque bouton = low signifie actionner
(2026-03-11) par Cloé -> Fonctionnel

4. Test de la pin interrup
(2026-03-25 à 12h) Marco et Cloé -> S'active seulement quand deux bouton appuyer

*/

//BOUTONS
#define BTN_PIN_UP 22
#define BTN_PIN_DOWN 25
#define BTN_PIN_LEFT 23
#define BTN_PIN_RIGHT 24
#define BTN_PIN_OK 26
#define BTN_Interup 3
#define DEBOUNCE_MS 20

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(BTN_PIN_UP, INPUT_PULLUP); //Verifier avec Marcob input vs input_pullup (resistance interne ou pas)
  pinMode(BTN_PIN_DOWN, INPUT_PULLUP);
  pinMode(BTN_PIN_LEFT, INPUT_PULLUP);
  pinMode(BTN_PIN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_PIN_OK, INPUT_PULLUP);
  pinMode(BTN_Interup, INPUT_PULLUP);
}

void loop() {  
  // 1.
  Serial.print(digitalRead(BTN_PIN_UP));
  Serial.println("  Vert");
  
  // 2.
  Serial.print(digitalRead(BTN_PIN_DOWN));
  Serial.println("  Jaune");

  Serial.print(digitalRead(BTN_PIN_LEFT));
  Serial.println("  Bleu");

  Serial.print(digitalRead(BTN_PIN_RIGHT));
  Serial.println("  Rouge");

  Serial.print(digitalRead(BTN_PIN_OK));
  Serial.println("  Blanc");

  // 3.
  if(digitalRead(BTN_Interup) == LOW) {
    Serial.println("Interupt");
  }
  Serial.println("------------");

  delay(2000);
}