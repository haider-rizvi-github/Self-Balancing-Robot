#include <stdint.h>
#include "pins.h"

void voltage_init(uint16_t VOL_MEASURE_PIN) {
  pinMode(VOL_MEASURE_PIN, INPUT);
}

float voltage_read(){
  return analogRead(VOL_MEASURE_PIN);
}