// This sketch will check if the NFC pins are configured for NFC mode,
// and if so it will switch them to operate in GPIO mode. A system
// reset is required before this change takes effect since the CONFIG
// memory is only read on power up.

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!! IMPORTANT NOTE ... READ BEFORE RUNNING THIS SKETCH !!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// UICR customer registers are meant to be stored with values
// that are supposed to stay there during the life time of the softdevice.
// You cannot erase them without erasing everything on chip, so setting the
// NFC pins to GPIO mode is a ONE WAY OPERATION and you will need a debugger
// like a Segger J-Link to set them back to NFC mode!

void setup() {
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  
  Serial.println("Bluefruit52 NFC to GPIO Pin Config");
  Serial.println("----------------------------------\n");
      if ((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) == (UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos)){
        Serial.println("Fix NFC pins");
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
        NRF_UICR->NFCPINS &= ~UICR_NFCPINS_PROTECT_Msk;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
        Serial.println("Done");
        delay(500);
        NVIC_SystemReset();
      }

}
void loop() {
  // put your main code here, to run repeatedly:
}
