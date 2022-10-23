/*
 This is example usage of the nrfx_temp driver
*/

#include <nrfx_temp.h>
#include <Adafruit_TinyUSB.h> 

nrfx_temp_config_t config = NRFX_TEMP_DEFAULT_CONFIG;

// this function is called when the temperature measurement is ready
void temp_handler(int32_t raw_temp) {
  Serial.println(nrfx_temp_calculate(raw_temp));
}

void setup() {
  Serial.begin(9600);
  nrfx_temp_init(&config, &temp_handler);
}

void loop() {
  nrfx_temp_measure();
  
  delay(1000);
}
