/*
 * Very Low Power Toothbrush Timer
 * 
 * This source file is part of the follwoing repository:
 * http://www.github.com/microfarad-de/brush-timer
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



/*
 * Pin assignment
 */
#define POWER_MOSFET_PIN     A2      // Pin that controls the power-on transistor
#define POWER_BUTTON_PIN     A3      // Pin that senses the power button state    


/*           
 * LED pins:
 * 
 *           E   C
 *    H              A
 *       G      
 *    I              B
 *           F   D
 */
#define A_PIN   12
#define B_PIN   7
#define C_PIN   9
#define D_PIN   6
#define E_PIN   8
#define F_PIN   5
#define G_PIN   2
#define H_PIN   4
#define I_PIN   3 


/*
 * Macros
 */
#define NUM_LEDS           9  // Total number of LEDs
#define POWER_OFF_DELAY    3  // Time duration in s for pressing the power button until the system is powered off
#define TIMER_DURATION   180  // Countdown timer duration in seconds
#define BLINK_DURATION   100  // LED blink duration in ms


/*
 * Global variables structure
 */
struct {
  uint8_t  ledPin[NUM_LEDS]   = { A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, F_PIN, G_PIN, H_PIN, I_PIN  };  // Array of LED pins
  uint8_t  ledState[NUM_LEDS] = { LOW };  // LED power-on states
  uint32_t secondsElapsed = 1;            // Countdown timer elapsed seconds
} G;



/*
 * Arduino initialization routine
 */
void setup () {
  // Disable the watchdog in case of a reebot due to watchdog expiry
  MCUSR = 0;      // clear MCU status register
  wdt_disable (); // and disable watchdog

  // Initialize I/O pins
  pinMode (POWER_MOSFET_PIN, OUTPUT);
  pinMode (POWER_BUTTON_PIN, INPUT_PULLUP);
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    pinMode (G.ledPin[i], OUTPUT);
  }

  // Turn off all HW peripherals except Timer 0
  // Timer 0 is used for generating the interrupt for millis()
  power_all_disable ();
  power_timer0_enable ();

  
  // Enable watchdog interrupt, set to 1s (see Datasheet Section 15.9.2)
  // The watchdog interrupt will wake-up the CPU from deep sleep once per second
  // and will serve as the maine timekeeping clock source.
  cli ();
  WDTCSR |= _BV(WDCE) | _BV(WDE);
  WDTCSR = _BV(WDIE) | _BV(WDP2) | _BV(WDP1); 
  sei ();

}


/*
 * Arduino main loop
 */
void loop () {
  // Main state machine states
  static enum { STATE_INIT, STATE_COUNTDOWN, STATE_COUNTDOWN_B, STATE_FINISH, STATE_DEEPSLEEP, STATE_SHUTDOWN } state = STATE_INIT;
  static uint32_t blinkTs = 0;
  static uint32_t buttonPressSecs = 0;
  uint32_t ts = millis ();

  
  // Call the LED multiplexing routine
  muxLeds ();


  // Main state machine
  switch (state) {

    /*
     * Initialization State
     * Power on the system
     */
    case STATE_INIT:
      // Power on the system
      digitalWrite (POWER_MOSFET_PIN, HIGH);

      state = STATE_COUNTDOWN;
    
      break;


    /*
     * Countdown Timer State
     * Update the LED status every second LEDs
     */
    case STATE_COUNTDOWN:
      blinkTs = ts;
      state = STATE_COUNTDOWN_B;
      
    case STATE_COUNTDOWN_B:
      G.ledState[0] = HIGH;
      G.ledState[1] = HIGH;
      
      if (G.secondsElapsed >= TIMER_DURATION / 4) {
        G.ledState[2] = HIGH;
        G.ledState[3] = HIGH;
      }
      if (G.secondsElapsed >= 2 * TIMER_DURATION / 4) {
        G.ledState[4] = HIGH;
        G.ledState[5] = HIGH;           
      }
      if (G.secondsElapsed >= 3 * TIMER_DURATION / 4) {
        G.ledState[6] = HIGH;
      }
      if (G.secondsElapsed >= TIMER_DURATION) {
        setLedStates (LOW, true);
        state = STATE_FINISH;      
      }


      // Blink duration elapsed - turn off all LEDs and go to deep sleep
      if (ts - blinkTs > BLINK_DURATION) {
        setLedStates (LOW, true);
        state = STATE_DEEPSLEEP;
      }
      
      lightSleep ();
      
      break;
  

    /*
     * Finish State
     * Execute final LED blinking sequence prior to shutdown
     */
    case STATE_FINISH:
      if ( animate1 () ) {
        setLedStates (LOW, true);
        state = STATE_SHUTDOWN;
      }

      lightSleep ();

      break;


    /*
     * Deep Sleep State
     * Power down the CPU and wait for the next watchdog interrupt
     * Increment the seconds counter after every wake-up
     */
    case STATE_DEEPSLEEP:
      // Power down the CPU
      deepSleep();
      // Increment elapsed seconds upon wake-up 
      G.secondsElapsed++;

      // Detect long power button press to power down the system
      if (digitalRead (POWER_BUTTON_PIN) == LOW) {
        buttonPressSecs++;
        if (buttonPressSecs >= POWER_OFF_DELAY) {
          state = STATE_FINISH;
          break;
        }
      }
      else {
        buttonPressSecs = 0;                           
      }
      
      state = STATE_COUNTDOWN;
      
      break;


    /*
     * Shutdown State
     * Turn the power off
     */
    case STATE_SHUTDOWN:

      // Turn off the power supply
      digitalWrite (POWER_MOSFET_PIN, LOW);
      
      // Stay here until power is off
      while (1);       
      
      break;


    default:
      break;
    
  }

}


/*
 * LED multiplexing routine
 * Since all the LEDs share one common dropper resistor, they must be driven 
 * via multiplexing. This means that the LEDs are sequentially activated
 * fast enough to create the illusion that they are lit simultanously for the 
 * human eye. This way we save ourselves 8 dropper resistors.
 */
void muxLeds () {
  static uint8_t idx = 0;

  digitalWrite (G.ledPin[idx], LOW);

  idx ++;
  if (idx >= NUM_LEDS) idx = 0;
  
  digitalWrite (G.ledPin[idx], G.ledState[idx]);
  
}


/*
 * Turna all LEDs on or off
 */
void setLedStates ( uint8_t state, bool apply ) {
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    G.ledState[i] = state;
    if (apply) digitalWrite (G.ledPin[i], state);   
  }
}



/*
 * LED animation sequence
 * Returns true when animation sequence is finished
 */
bool animate1 () {
  static uint32_t blinkTs = 0;
  static uint32_t count = 0;
  static uint8_t state = HIGH;
  uint32_t ts = millis ();

  if ( (ts - blinkTs > 200) || (ts - blinkTs > 200) ){
    setLedStates (state, false);
    state = !state;
    blinkTs = ts;
    count++;
  }

  if (count > 30) return true;
  else            return false;
}


/*
 * Send the CPU into light sleep mode
 * The CPU will be waken-up by the 1ms millis() Timer 0 interrupt
 */
void lightSleep () {
  set_sleep_mode (SLEEP_MODE_IDLE);     // Configure lowest sleep mode that keeps clk_IO for Timer 0
  sleep_enable ();                      // Prepare for sleep
  sleep_cpu ();                         // Send the CPU into seelp mode
  sleep_disable ();                     // CPU will wake-up here
}



/*
 * Send the CPU into deep sleep mode
 * The CPU will be waken-up by the 1s watchdog interrupt
 */
void deepSleep () {
  set_sleep_mode (SLEEP_MODE_PWR_DOWN); // Configure lowest possible sleep mode
  cli ();
  sleep_enable ();                      // Prepare for sleep
  sleep_bod_disable ();                 // Disable brown-out detection (only for SLEEP_MODE_PWR_DOWN)
  sei ();
  sleep_cpu ();                         // Send the CPU into seelp mode
  sleep_disable ();                     // CPU will wake-up here
}



/*
 * Watchdog interrupt service routine
 */
ISR (WDT_vect)  {
  // Do nothing
}
