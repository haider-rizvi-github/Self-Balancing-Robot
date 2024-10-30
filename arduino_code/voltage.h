#include "Arduino.h"
#include <stdint.h>
#include "pins.h"

unsigned long vol_measure_time = 0;

void voltage_init() {
  analogReference(INTERNAL);
}

float voltage_read() {
  if (millis() - vol_measure_time > 1000) {
    vol_measure_time = millis();
    return (analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5);
  }
}