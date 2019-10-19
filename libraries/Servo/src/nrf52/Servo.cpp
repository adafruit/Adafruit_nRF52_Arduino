/*
  Copyright (c) 2016 Arduino. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#if defined(ARDUINO_ARCH_NRF52) || defined(ARDUINO_NRF52_ADAFRUIT)

#include <Arduino.h>
#include <Servo.h>


static servo_t servos[MAX_SERVOS];              // static array of servo structures
uint8_t ServoCount = 0;                         // the total number of attached servos

Servo::Servo()
{
  if (ServoCount < MAX_SERVOS) {
    this->servoIndex = ServoCount++;            // assign a servo index to this instance
  } else {
    this->servoIndex = INVALID_SERVO;  					// too many servos
  }

  this->pwm = NULL;
}

uint8_t Servo::attach(int pin)
{
	return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t Servo::attach(int pin, int min, int max)
{
  if (this->servoIndex < MAX_SERVOS) {
    pinMode(pin, OUTPUT);                                   // set servo pin to output
    servos[this->servoIndex].Pin.nbr = pin;

    if (min < MIN_PULSE_WIDTH) min = MIN_PULSE_WIDTH;
    if (max > MAX_PULSE_WIDTH) max = MAX_PULSE_WIDTH;

    //fix min if conversion to pulse cycle value is too low
    if((min/DUTY_CYCLE_RESOLUTION)*DUTY_CYCLE_RESOLUTION<min) min+=DUTY_CYCLE_RESOLUTION;
	  
    this->min  = min;
    this->max  = max;

    servos[this->servoIndex].Pin.isActive = true;

    // Adafruit, add pin to 1 of available Hw PWM
    for(int i=0; i<HWPWM_MODULE_NUM; i++)
    {
      if ( HwPWMx[i]->addPin(pin) )
      {
        this->pwm = HwPWMx[i];
        break;
      }
    }

    this->pwm->setMaxValue(MAXVALUE);
    this->pwm->setClockDiv(CLOCKDIV);

  }
  return this->servoIndex;
}

void Servo::detach()
{
  uint8_t const pin = servos[this->servoIndex].Pin.nbr;

	servos[this->servoIndex].Pin.isActive = false;

	// remove pin from HW PWM
	this->pwm->removePin(pin);
}


void Servo::write(int value)
{  
	if (value < 0)
		value = 0;
	else if (value > 180)
		value = 180;
	value = map(value, 0, 180, this->min, this->max);
	
	writeMicroseconds(value);
}


void Servo::writeMicroseconds(int value)
{
	uint8_t pin = servos[this->servoIndex].Pin.nbr;
	
	if ( this->pwm ) this->pwm->writePin(pin, value/DUTY_CYCLE_RESOLUTION);
}

int Servo::read() // return the value as degrees
{
	return map(readMicroseconds(), this->min, this->max, 0, 180);
}

int Servo::readMicroseconds()
{	
	uint8_t pin = servos[this->servoIndex].Pin.nbr;

	if ( this->pwm ) return this->pwm->readPin(pin)*DUTY_CYCLE_RESOLUTION;

	return 0;
}

bool Servo::attached()
{
  return servos[this->servoIndex].Pin.isActive;
}

#endif // ARDUINO_ARCH_NRF52
