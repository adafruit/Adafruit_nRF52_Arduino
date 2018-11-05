/*********************************************************************
 This is an example for our Feather Bluefruit modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/*
 * This sketch use all 12 channels of 3 Hardware PWM.
 * Also change the resolution to max 15-bit.
 * Each PWM Hardware group (4 pin) also run with different frequency
 * - PWM0 : clock/1  ~ 16Mhz
 * - PWM1 : clock/4  ~ 4Mhz
 * - PMW2 : clock/16 ~ 1Mhz
 * 
 * Since LED RED and BLUE are on different Hardware PWM, 
 * LED BLUE will blink (due to lower freq) while fading while LED RED
 * looks more solid. Furthermore LED RED group write value (PWM1) is inverted 
 * compared to LED BLUE (PWM2) --> They fade in opposite direction. 
 */

#include <Arduino.h>

// Maximum 12 pins can be used for 3 PWM modules ( 4 channel each )
// Note: nRF52832 has 3 PWM modules, nRF52840 has 4 PWM modules
int pins[12] = 
{ 
  A0 , A1      , A2          , A3,
  A4 , A5      , A6          , LED_RED, /* avoid A7 (VBAT)  */
  27 , LED_BLUE, PIN_WIRE_SDA, PIN_WIRE_SCL
};

/**************************************************************************/
/*!
    @brief  The setup function runs once when reset the board
*/
/**************************************************************************/
void setup()
{
  // Add 4 pins into a group
  // It is better to add Pin before call .begin()
  for (int i=0; i<12; i++)
  {
    HwPWMx[i/4]->addPin( pins[i] );
  }

  // Enable all 3 PWM modules with 15-bit resolutions(max) but different clock div
  HwPWM0.begin();
  HwPWM0.setResolution(15);
  HwPWM0.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_1); // default : freq = 16Mhz
  
  HwPWM1.begin();
  HwPWM1.setResolution(15);
  HwPWM1.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_4); // default : freq = 4Mhz
  
  HwPWM2.begin();
  HwPWM2.setResolution(15);
  HwPWM2.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_16); // default : freq = 1Mhz
}

/**************************************************************************/
/*!
    @brief  The loop function runs over and over again forever
*/
/**************************************************************************/
void loop()
{
  const int maxValue = bit(15) - 1;
  bool inverted;
  
  // fade in from min to max
  // inverted PWM0 (false), PWM1 (true), PWM2 (false)
  for (int fadeValue = 0 ; fadeValue <= maxValue; fadeValue += 1024) 
  {
    inverted = false;
    
    for (int i=0; i<12; i++)
    {
      // Inverted for each PWM group
      if (i%4)  inverted = !inverted; 
      
      HwPWMx[i/4]->writePin( pins[i], fadeValue, inverted);
    }
    
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }

  // fade out from max to min
  // inverted PWM0 (false), PWM1 (true), PWM2 (false)
  for (int fadeValue = maxValue ; fadeValue >= 0; fadeValue -= 1024) 
  {
    inverted = false;
     
    for (int i=0; i<12; i++)
    {
      // Inverted for each PWM group
      if (i%4)  inverted = !inverted; 
      
      HwPWMx[i/4]->writePin( pins[i], fadeValue, inverted);
    }
    
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }
}
