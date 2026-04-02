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

  float d1 = interp_adc_to_mm(LUT1_adc, LUT1_mm, sizeof(LUT1_adc)/sizeof(LUT1_adc[0]), ir1) + 27.5f;
  float d2 = interp_adc_to_mm(LUT2_adc, LUT2_mm, sizeof(LUT2_adc)/sizeof(LUT2_adc[0]), ir2) + 44.0f;
  float d3 = interp_adc_to_mm(LUT3_adc, LUT3_mm, sizeof(LUT3_adc)/sizeof(LUT3_adc[0]), ir3) + 44.0f;
  float d4 = interp_adc_to_mm(LUT4_adc, LUT4_mm, sizeof(LUT4_adc)/sizeof(LUT4_adc[0]), ir4) + 27.5f;

  Serial.printf("%.1f,%.1f,%.1f,%.1f\n", d1, d2, d3, d4);
}

#endif
