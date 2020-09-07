#include <Wire.h>

TwoWire *wi = &Wire;
//TwoWire *wi = &Wire1;

void setup()
{
  Serial.begin(115200);  // start serial for output
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  wi->begin();        // join i2c bus (address optional for main)
}

void loop()
{
  Serial.println("Scanning address from 0 to 127");
  for (int addr = 1; addr < 128; addr++)
  {
    wi->beginTransmission(addr);
    if ( 0 == wi->endTransmission() )
    {
      Serial.print("Found: 0x");
      Serial.print(addr, HEX);
      Serial.println();
    }
  }

  delay(5000);
}
