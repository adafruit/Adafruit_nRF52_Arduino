#include <bluefruit.h>

typedef volatile uint32_t REG32;
#define pREG32 (REG32 *)

#define DEVICE_ID_HIGH    (*(pREG32 (0x10000060)))
#define DEVICE_ID_LOW     (*(pREG32 (0x10000064)))
#define MAC_ADDRESS_HIGH  (*(pREG32 (0x100000a8)))
#define MAC_ADDRESS_LOW   (*(pREG32 (0x100000a4)))

void setup() {
  Serial.begin(115200);

  Serial.println("Bluefruit 52 HW Info");
  Serial.println("");

  // MAC Address
  uint32_t addr_high = ((MAC_ADDRESS_HIGH) & 0x0000ffff) | 0x0000c000;
  uint32_t addr_low  = MAC_ADDRESS_LOW;
  Serial.print("MAC Address: ");
  Serial.print((addr_high >> 8) & 0xFF, HEX); Serial.print(":");
  Serial.print((addr_high) & 0xFF, HEX); Serial.print(":");
  Serial.print((addr_low >> 24) & 0xFF, HEX); Serial.print(":");
  Serial.print((addr_low >> 16) & 0xFF, HEX); Serial.print(":");
  Serial.print((addr_low >> 8) & 0xFF, HEX); Serial.print(":");
  Serial.print((addr_low) & 0xFF, HEX); Serial.println("");

  // Unique Device ID
  Serial.print("Device ID:   ");
  Serial.print(DEVICE_ID_HIGH, HEX);
  Serial.println(DEVICE_ID_LOW, HEX);
}

void loop() {
  // put your main code here, to run repeatedly:

}
