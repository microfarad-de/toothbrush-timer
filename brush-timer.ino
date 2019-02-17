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
 * LED object
 */
LedClass Led;

/*
 * Arduino initialization routine
 */
void setup () {
  // Disable the watchdog in case of a reebot due to watchdog expiry
  MCUSR = 0;      // clear MCU status register
  wdt_disable (); // and disable watchdog

  // Initialize the LED object
  Led.initialize (LED_PIN);
  Led.blink (-1, 100, 1900);

}


/*
 * Arduino main loop
 */
void loop () {

  // Update the LED status
  Led.loopHandler ();

  // Send the CPU into sleep mode
  powerSave();

}



/*
 * Enter the power save mode
 */
void powerSave () {
  power_all_disable ();                 // Turn off peripherals
  power_timer0_enable ();               // Power on Timer 1
  set_sleep_mode (SLEEP_MODE_IDLE);   // Configure lowest sleep mode that keeps clk_IO for Timer 1
  //set_sleep_mode (SLEEP_MODE_PWR_DOWN); // Configure lowest possible sleep mode
  //cli ();
  sleep_enable ();                      // Prepare for sleep
  //sleep_bod_disable ();               // Disable brown-out detection (only for SLEEP_MODE_PWR_DOWN)
  //sei ();
  sleep_cpu ();                         // Send the CPU into seelp mode
  sleep_disable ();                     // CPU will wake-up here
}
