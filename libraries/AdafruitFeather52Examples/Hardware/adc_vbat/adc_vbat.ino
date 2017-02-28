#define VBAT_PIN          (A7)
#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider

float readVBAT(void) {
  float raw;

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(VBAT_PIN);

  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  // Convert the raw value to mv
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
  // VBAT voltage divider is 2M + 0.806M, which needs to be added back
  return raw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP;
}

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Get a fresh ADC value
  float vbat_val = readVBAT();

  // Display the results
  Serial.print(vbat_val);
  Serial.println(" mV");

  delay(1000);
}
