/*
 This is example usage of the nrfx_temp driver
*/

#include <nrfx_temp.h>
#include <Adafruit_TinyUSB.h> 

nrfx_temp_config_t config = NRFX_TEMP_DEFAULT_CONFIG;

void setup() {
  Serial.begin(9600);
  // setting the handler to NULL will initialize the driver in blocking mode
  nrfx_temp_init(&config, NULL);
}

void loop() {
  nrfx_temp_measure(); // In blocking mode: this function waits until the measurement is finished.
  int32_t raw_temp = nrfx_temp_result_get();
  Serial.println(nrfx_temp_calculate(raw_temp));

  delay(1000);
}
