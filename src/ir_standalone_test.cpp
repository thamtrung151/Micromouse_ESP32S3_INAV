// Standalone IR reader test (reference).
//
// Define IR_STANDALONE_TEST to build this as the firmware entry point.
// In normal builds, this compiles to nothing.

#ifdef IR_STANDALONE_TEST

#include <Arduino.h>
#include "config.h"

static constexpr uint32_t PRINT_HZ = 200; // prints CSV at 200Hz
static constexpr uint32_t PERIOD_MS = 1000 / PRINT_HZ;

void setup() {
  Serial.begin(115200);
  delay(200);

  analogReadResolution(IR_ADC_BITS);
  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  pinMode(IR3_PIN, INPUT);
  pinMode(IR4_PIN, INPUT);

  Serial.println("IR standalone: CSV ir1,ir2,ir3,ir4");
}

void loop() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < PERIOD_MS) return;
  last = now;

  uint16_t ir1 = (uint16_t)analogRead(IR1_PIN);
  uint16_t ir2 = (uint16_t)analogRead(IR2_PIN);
  uint16_t ir3 = (uint16_t)analogRead(IR3_PIN);
  uint16_t ir4 = (uint16_t)analogRead(IR4_PIN);

  Serial.printf("%u,%u,%u,%u\n", ir1, ir2, ir3, ir4);
}

#endif
