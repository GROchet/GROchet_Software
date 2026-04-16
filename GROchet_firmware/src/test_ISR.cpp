#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include "event_groups.h"

// -------- BUTTON PINS --------
#define BTN_PIN_UP     22
#define BTN_PIN_DOWN   25
#define BTN_PIN_LEFT   24
#define BTN_PIN_RIGHT  23
#define BTN_PIN_OK     26

#define PIN_BTN_INTERRUPT 3

// -------- EVENT BITS --------
#define EVT_BTN_OK    (1 << 0)
#define EVT_BTN_UP    (1 << 1)
#define EVT_BTN_DOWN  (1 << 2)
#define EVT_BTN_LEFT  (1 << 3)
#define EVT_BTN_RIGHT (1 << 4)

// -------- DEBOUNCE --------
#define DEBOUNCE_MS 20

volatile TickType_t lastBtnInterrupt = 0;

EventGroupHandle_t inputEventGroup;

// -------- ISR --------
void NotifyBtn() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    bool up_state    = digitalRead(BTN_PIN_UP) == LOW;
    bool down_state  = digitalRead(BTN_PIN_DOWN) == LOW;
    bool left_state  = digitalRead(BTN_PIN_LEFT) == LOW;
    bool right_state = digitalRead(BTN_PIN_RIGHT) == LOW;
    bool ok_state    = digitalRead(BTN_PIN_OK) == LOW;

    // OK overrides everything
    if (ok_state) {
        xEventGroupSetBitsFromISR(inputEventGroup, EVT_BTN_OK, &xHigherPriorityTaskWoken);
        xEventGroupClearBitsFromISR(inputEventGroup, EVT_BTN_UP   | EVT_BTN_DOWN | EVT_BTN_LEFT | EVT_BTN_RIGHT);
    } else {
        xEventGroupClearBitsFromISR(inputEventGroup, EVT_BTN_OK);

        if (up_state)    xEventGroupSetBitsFromISR(inputEventGroup, EVT_BTN_UP, &xHigherPriorityTaskWoken);
        else             xEventGroupClearBitsFromISR(inputEventGroup, EVT_BTN_UP);

        if (down_state)  xEventGroupSetBitsFromISR(inputEventGroup, EVT_BTN_DOWN, &xHigherPriorityTaskWoken);
        else             xEventGroupClearBitsFromISR(inputEventGroup, EVT_BTN_DOWN);

        if (left_state)  xEventGroupSetBitsFromISR(inputEventGroup, EVT_BTN_LEFT, &xHigherPriorityTaskWoken);
        else             xEventGroupClearBitsFromISR(inputEventGroup, EVT_BTN_LEFT);

        if (right_state) xEventGroupSetBitsFromISR(inputEventGroup, EVT_BTN_RIGHT, &xHigherPriorityTaskWoken);
        else             xEventGroupClearBitsFromISR(inputEventGroup, EVT_BTN_RIGHT);
    }

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// -------- TASK TO MONITOR EVENTS --------
void TaskMonitor(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    EventBits_t bits = xEventGroupWaitBits(
      inputEventGroup,
      EVT_BTN_OK | EVT_BTN_UP | EVT_BTN_DOWN | EVT_BTN_LEFT | EVT_BTN_RIGHT,
      pdTRUE,     // clear on exit
      pdFALSE,
      portMAX_DELAY
    );

    Serial.print("EVENT RECEIVED: ");

    if (bits & EVT_BTN_OK)    Serial.print("OK ");
    if (bits & EVT_BTN_UP)    Serial.print("UP ");
    if (bits & EVT_BTN_DOWN)  Serial.print("DOWN ");
    if (bits & EVT_BTN_LEFT)  Serial.print("LEFT ");
    if (bits & EVT_BTN_RIGHT) Serial.print("RIGHT ");

    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  inputEventGroup = xEventGroupCreate();

  pinMode(BTN_PIN_UP, INPUT_PULLUP);
  pinMode(BTN_PIN_DOWN, INPUT_PULLUP);
  pinMode(BTN_PIN_LEFT, INPUT_PULLUP);
  pinMode(BTN_PIN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_PIN_OK, INPUT_PULLUP);

  pinMode(PIN_BTN_INTERRUPT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_BTN_INTERRUPT), NotifyBtn, FALLING);

  xTaskCreate(TaskMonitor, "Monitor", 256, NULL, 1, NULL);

  Serial.println("FULL ISR TEST READY");
}

void loop() {
  // RTOS handles everything
}