/*
 * Very Low Power Toothbrush Timer
 * 
 * This source file is part of the Nixie Clock Arduino firmware
 * found under http://www.github.com/microfarad-de/brush-timer
 * 
 * Please visit:
 *   http://www.microfarad.de
 *   http://www.github.com/microfarad-de 
 *   
 * Copyright (C) 2019 Karim Hraibi (khraibi@gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Version: 1.0.0
 * Date:    February 2019
 */
#define VERSION_MAJOR 1  // Major version
#define VERSION_MINOR 0  // Minor version
#define VERSION_MAINT 0  // Maintenance version


#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <Arduino.h>
#include "helper.h"



/*
 * Pin assignment
 */
#define POWER_CONTROL_PIN     11      // Pin that controls the power-on transistor
#define POWER_BUTTON_PIN      12      // Pin that senses the power button state     
#define LED_PIN               13      // LED pin 

/*
 * Arduino initialization routine
 */
void setup() {
  MCUSR = 0;      // clear MCU status register
  wdt_disable (); // and disable watchdog

}


/*
 * Arduino main loop
 */
void loop() {
  // put your main code here, to run repeatedly:

}
