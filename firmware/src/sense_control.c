/*
  sense_control.h - sensing and controlling inputs and outputs
  Part of LasaurGrbl

  Copyright (c) 2011 Stefan Hechenberger

  LasaurGrbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  LasaurGrbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include "sense_control.h"
#include "stepper.h"
#include "planner.h"



void sense_init() {
  //// chiller, door, (power)
  SENSE_DDR &= ~(SENSE_MASK);  // set as input pins 
  // SENSE_PORT |= SENSE_MASK;    //activate pull-up resistors 
  
  //// x1_lmit, x2_limit, y1_limit, y2_limit, z1_limit, z2_limit
  LIMIT_DDR &= ~(LIMIT_MASK);  // set as input pins
  // LIMIT_PORT |= LIMIT_MASK;    //activate pull-up resistors   
}


void control_init() {
  //// laser control
  // Setup Timer0.
  // Timer0 can pwm either PD5 (OC0B) or PD6 (OC0A), we use PD6
  //
  // PD6 is wired to enable/disable the laser.  The other PSU input
  // which allows to change the output current (via resistor or fast
  // PWM) is not under our control.  The Coletech 100W PSU specifies
  // the response time (to reach nominal current) to be <=1ms, which
  // gives a rough idea about the minimum pulse duration.
  //
  // TCCR0A and TCCR0B are the registers to setup Timer0
  // see chapter "8-bit Timer/Counter0 with PWM" in Atmga328 specs
  // OCR0A sets the duty cycle 0-255 corresponding to 0-100%
  // also see: http://arduino.cc/en/Tutorial/SecretsOfArduinoPWM
  //
  // Timer0 is used to fully enable or disable the laser for a whole
  // timer period, rather than as a PWM. This allows to control
  // precisely the position where a pulse starts in raster mode.
  //
  // Note: An alternative would be to use the AVR timer to create
  // one-shot pulses, but we'd have to change the output pin to PD5:
  // http://hackaday.com/2015/03/24/avr-hardware-timer-tricked-into-one-shot/

  DDRD |= (1 << DDD6);    // set PD6 as an output
  OCR0A = 0;              // disable
  // Control register setup:
  // - phase-correct PWM (because it allows both always-on and always-off)
  // - OC0A: output (non-inverted)
  // - OC0B: disconnected
  // - prescaler: 1 (options: 1, 8, 64, 256, 1024)
  // - period = prescaler*510/16Mhz = 31.875us
  TCCR0A = _BV(COM0A1) | _BV(WGM00);
  TCCR0B = _BV(CS00);
  TIMSK0 |= _BV(TOIE0); // Enable Timer0 overflow interrupt

  //// air and aux assist control
  ASSIST_DDR |= (1 << AIR_ASSIST_BIT);   // set as output pin
  ASSIST_DDR |= (1 << AUX1_ASSIST_BIT);  // set as output pin
  control_air_assist(false);
  control_aux1_assist(false);
  ASSIST_DDR |= (1 << AUX2_ASSIST_BIT);  // set as output pin
  control_aux2_assist(false);  
}


static volatile uint8_t pulse_remaining = 0;
static volatile uint16_t next_pulse_delay = 0;
static volatile uint8_t next_pulse_duration = 0;

void control_laser_pulse(uint8_t duration, uint16_t delay) {
  cli();
  next_pulse_delay = delay + 1;
  next_pulse_duration = duration;
  sei();
}

// Laser ISR
ISR(TIMER0_OVF_vect) {
  if (next_pulse_delay > 0) {
    next_pulse_delay--;
    if (next_pulse_delay == 0) {
      pulse_remaining = next_pulse_duration;
    }
  }
  if (pulse_remaining > 0) {
    OCR0A = 255;
    pulse_remaining--;
  }  else {
    OCR0A = 0;
  }
}

void control_air_assist(bool enable) {
  if (enable) {
    ASSIST_PORT |= (1 << AIR_ASSIST_BIT);
  } else {
    ASSIST_PORT &= ~(1 << AIR_ASSIST_BIT);
  }
}

void control_aux1_assist(bool enable) {
  if (enable) {
    ASSIST_PORT |= (1 << AUX1_ASSIST_BIT);
  } else {
    ASSIST_PORT &= ~(1 << AUX1_ASSIST_BIT);
  }  
}

void control_aux2_assist(bool enable) {
  if (enable) {
    ASSIST_PORT |= (1 << AUX2_ASSIST_BIT);
  } else {
    ASSIST_PORT &= ~(1 << AUX2_ASSIST_BIT);
  }  
}
