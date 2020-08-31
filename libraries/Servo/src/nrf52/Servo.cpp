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

  Modified by Ha Thach for Adafruit Industries
  Modified by Henry Gabryjelski to add ownershp support
*/

#if defined(ARDUINO_ARCH_NRF52) || defined(ARDUINO_NRF52_ADAFRUIT)

#include <Arduino.h>
#include <Servo.h>

enum
{
  SERVO_TOKEN = 0x76726553 // 'S' 'e' 'r' 'v'
};

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
  if (this->servoIndex == INVALID_SERVO) {
    return INVALID_SERVO;
  }
  bool succeeded = false;

  if (min < MIN_PULSE_WIDTH)
  {
    min = MIN_PULSE_WIDTH;
  }

  if (max > MAX_PULSE_WIDTH)
  {
    max = MAX_PULSE_WIDTH;
  }

  //fix min if conversion to pulse cycle value is too low
  if ( (min / DUTY_CYCLE_RESOLUTION) * DUTY_CYCLE_RESOLUTION < min )
  {
    min += DUTY_CYCLE_RESOLUTION;
  }
  
  this->min = min;
  this->max = max;

  // Adafruit, add pin to 1 of available Hw PWM
  // first, use existing HWPWM modules (already owned by Servo)
  for ( int i = 0; i < HWPWM_MODULE_NUM; i++ )
  {
    if ( HwPWMx[i]->isOwner(SERVO_TOKEN) && HwPWMx[i]->addPin(pin) )
    {
      this->pwm = HwPWMx[i];
      succeeded = true;
      break;
    }
  }

  // if could not add to existing owned PWM modules, try to add to a new PWM module
  if ( !succeeded )
  {
    for ( int i = 0; i < HWPWM_MODULE_NUM; i++ )
    {
      if ( HwPWMx[i]->takeOwnership(SERVO_TOKEN) && HwPWMx[i]->addPin(pin) )
      {
        this->pwm = HwPWMx[i];
        succeeded = true;
        break;
      }
    }
  }

  if ( succeeded )
  {
    pinMode(pin, OUTPUT);
    servos[this->servoIndex].Pin.nbr = pin;
    servos[this->servoIndex].Pin.isActive = true;

    this->pwm->setMaxValue(MAXVALUE);
    this->pwm->setClockDiv(CLOCKDIV);

    return this->servoIndex;
  }else
  {
    return INVALID_SERVO;
  }
}

void Servo::detach()
{
  if (this->servoIndex == INVALID_SERVO) {
    return;
  }

  uint8_t const pin = servos[this->servoIndex].Pin.nbr;
  servos[this->servoIndex].Pin.isActive = false;

  // remove pin from HW PWM
  HardwarePWM * pwm = this->pwm;
  this->pwm = nullptr;
  pwm->removePin(pin);
  if (pwm->usedChannelCount() == 0) {
    pwm->stop(); // disables peripheral so can release ownership
    pwm->releaseOwnership(SERVO_TOKEN);
  }
}

void Servo::write(int value)
{
	if (value < 0) {
		value = 0;
  } else if (value > 180) {
		value = 180;
  }
	value = map(value, 0, 180, this->min, this->max);
	
	writeMicroseconds(value);
}


void Servo::writeMicroseconds(int value)
{
  if (this->pwm) {
	  uint8_t pin = servos[this->servoIndex].Pin.nbr;
  	this->pwm->writePin(pin, value/DUTY_CYCLE_RESOLUTION);
  }
}

int Servo::read() // return the value as degrees
{
	return map(readMicroseconds(), this->min, this->max, 0, 180);
}

int Servo::readMicroseconds()
{	
  if (this->pwm) {
    uint8_t pin = servos[this->servoIndex].Pin.nbr;
    return this->pwm->readPin(pin)*DUTY_CYCLE_RESOLUTION;
  }
	return 0;
}

bool Servo::attached()
{
  if (this->servoIndex == INVALID_SERVO) {
    return false;
  }
  return servos[this->servoIndex].Pin.isActive;
}

#endif // ARDUINO_ARCH_NRF52
