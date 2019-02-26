/*
 * Very Low Power Kids Toothbrush Timer
 * 
 * This Arduino sketch implements the firmware of a funny kids toothbrush timer.
 * 
 * The device features 9 LEDs arranged in a "little fish" pattern. When turned on, 
 * the eyes of the fish will start to blink once per second. More and more LEDs begin to 
 * blink as time passes. One of few random animation sequences is activated as soon as the 
 * recommended two minute tooth brushing duration elapses, then the system will power 
 * itself off.
 * 
 * The device implements a soft power on circuit using a tactile switch and two 
 * FET transistors. Briefly pressing the power button will power up the system and initiate
 * the timer sequence. Pressing the power button during aprroximately 2 seconds while the  
 * device is on will skip the timer sequence and directly activate the LED animation 
 * sequence prior to powering down the system.
 * 
 * The device is designed to consume a very low current and is able to run on a single CR2025 
 * or similar 3V Lithium cell for thousands of cycles. The current consumption is around 100μA 
 * during the deep sleep phase and about 1.5mA when all the LEDs are on. The device consumes no 
 * current when the power is off.
 * 
 * In order to reduce the bill of material, all 9 LEDs share one common dropper resistor.
 * Turning on multiple LEDs simultaneously is not recommended as it will result in some of 
 * the LEDs glowing brighter than others. A multiplexing routine is used for sequentially 
 * turning on one LED at a time and doing this fast enough to create the illusion that they 
 * are simultaneously lit due the the persistance of the human vision.
 * 
 * This sketch has been implemented and tested on an ATMega328P based Arduino Pro Mini 
 * compatible board running on 3.3V/8MHz.
 * 
 * It is recommended to activate the watchdog support on the Arduino bootloader
 * by defining the WATCHDOG_MODS macro. This will reduce the bootloader's power-up 
 * delay, thus invalidating the need to hold the power button for around 2 seconds for 
 * the system to turn on.
 * 
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
#define VERSION_MAINT 1  // Maintenance version



#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <Arduino.h>
#include <EEPROM.h>



//#define SERIAL_DEBUG              // Activate debug printing over RS232
#define SERIAL_BAUD 115200          // Serial baud rate

// use these macros for printing to serial port
#ifdef SERIAL_DEBUG  
  #define PRINT(...)   Serial.print   (__VA_ARGS__)
  #define PRINTLN(...) Serial.println (__VA_ARGS__)
#else
  #define PRINT(...)
  #define PRINTLN(...)
#endif



/*
 * Pin Assignment
 * Analog pins can also work as digital I/O pins
 */
#define POWER_MOSFET_PIN  A2  // Pin that controls the power-on FET transistor
#define POWER_BUTTON_PIN  A3  // Pin that senses the power button state (active LOW)
#define RANDOM_SEED_APIN  A0  // Floating analog pin for random seed generation



/*           
 * LED Pins
 * The pictogram below shows the LEDs arranged in a "little fish" pattern.
 * 
 *           E   C
 *    H              A
 *       G      
 *    I              B
 *           F   D
 */
#define A_PIN  12
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
#define NUM_LEDS                9  // Total number of LEDs
#define NUM_ANIM                5  // Total number of LED animation sequences
#define POWER_ON_DELAY        100  // Time duration in milliseconds for pressing the power button until the system is turned on
#define POWER_OFF_DELAY         2  // Time duration in seconds for pressing the power button until the system is turned off
#define TIMER_DURATION  (120 - 8)  // Countdown timer duration in seconds including the correction factor to compensate for the WDT inaccuracy
#define BLINK_DURATION        100  // LED blink duration in milliseconds
#define EEPROM_ADDR             0  // EEPROM base address



/*
 * LED Animation Function Prototypes
 */
bool animate1 (void);
bool animate2 (void);
bool animate3 (void);
bool animate4 (void);
bool animate5 (void);


/*
 * Global Variables
 */
struct {
  bool (*animate[NUM_ANIM])(void) = { animate1, animate2, animate3, animate4, animate5 };           // Array of animation sequences
  uint8_t  ledPin[NUM_LEDS]   = { A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, F_PIN, G_PIN, H_PIN, I_PIN };  // Array of LED pins
  uint8_t  ledState[NUM_LEDS] = { LOW };  // LED power-on states 
  uint32_t secondsElapsed     = 1;        // Countdown timer elapsed seconds
  uint8_t  animationIndex;                // Index of the chosen animation sequence
} G;



/*
 * Arduino Initialization Routine
 */
void setup () {
  // Ensure that the watchdog timer (WDT) has been disabled
  MCUSR = 0;       // Clear MCU status register
  wdt_disable ();  // Disable watchdog

#ifdef SERIAL_DEBUG  
  // initialize serial port
  Serial.begin (SERIAL_BAUD);
#endif

  PRINTLN ("+ + +  T O O T H B R U S H  T I M E R  + + +");

  // Initialize I/O pins
  pinMode (POWER_MOSFET_PIN, OUTPUT);
  pinMode (POWER_BUTTON_PIN, INPUT_PULLUP);
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    pinMode (G.ledPin[i], OUTPUT);
  }

  // Turn off all hardware peripherals except Timer 0
  // Timer 0 is used for generating the millisecond interrupt for the millis() function
  power_all_disable ();
  power_timer0_enable ();
#ifdef SERIAL_DEBUG  
  power_usart0_enable ();
#endif

  // Read and validate the last animation index from EEPROM
  G.animationIndex = EEPROM.read (EEPROM_ADDR);
  if (G.animationIndex >= NUM_ANIM) G.animationIndex = 0;

  /*
   * Enable watchdog interrupt with a 1 second interval.
   * The watchdog interrupt will wake-up the CPU from deep sleep once per second
   * and will serve as the main clock source for timekeeping.
   * The inaccuracy of the watchdog timer has been compensated within the TIMER_DURATION macro.
   * Please refer to the ATmega328P datasheet 15.9.2. "WDTCSR – Watchdog Timer Control Register"
   * for more information about configuring the WDT control register.
   */
  cli ();
  WDTCSR |= _BV(WDCE) | _BV(WDE);
  WDTCSR = _BV(WDIE) | _BV(WDP2) | _BV(WDP1); 
  sei ();

}



/*
 * Arduino Main Loop
 * 
 * All of the functions that are called from the Arduino main loop are non-blocking 
 * functions. This means that each function must spend minimal processing time and 
 * return as soon as possible. Upon return, each function has to remember its internal 
 * state and expect to be called again within 1 millisecond in order to resume its 
 * activity.
 * 
 * The above does not apply for the lightSleep() and deepSleep() functions which will
 * freeze the CPU and return within 1 millisecond and 1 second respectively.
 */
void loop () {
  static enum { STATE_INIT, STATE_INIT_B, STATE_COUNTDOWN, STATE_COUNTDOWN_B, 
      STATE_DEEPSLEEP, STATE_ANIMATE, STATE_ANIMATE_B, STATE_SHUTDOWN } state = STATE_INIT;  // Main state machine states
  static uint32_t blinkTs = 0;             // Timestamp for measuring the LED blink duration
  static uint32_t powerOnDelayTs = 0;      // Timestamp for measuring the power on delay
  static uint32_t buttonPressSecs = 0;     // Counts the seconds while the power button is pressed
  bool rv;                                 // General purpose variable
  uint32_t ts = millis ();                 // The current millisecond timestamp


  // Call the LED multiplexing routine
  muxLeds ();

  // Send the CPU into light sleep, will come back within 1 millisecond
  lightSleep ();


  /*
   * Main State Machine
   */
  switch (state) {

    /*
     * Initialization State
     * Power on the system
     */
    case STATE_INIT:
    
      powerOnDelayTs = ts;
      state = STATE_INIT_B;
      
    case STATE_INIT_B:
    
      // Wait some time while the power button is pressed
      if (ts - powerOnDelayTs > POWER_ON_DELAY) {
        // Power on the system
        digitalWrite (POWER_MOSFET_PIN, HIGH);
        state = STATE_COUNTDOWN;
      }
       
      break;


    /*
     * Countdown Timer State
     * Controls the LED blinking while progressiveliy activating more LEDs
     * as the timer count increases
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
        G.ledState[7] = HIGH;
        G.ledState[8] = HIGH;   
      }
      
      // Timer expired - turn off all LEDs and proceed to the animation routine
      if (G.secondsElapsed >= TIMER_DURATION + 1) {
        
        setLedStates (LOW, true);
        state = STATE_ANIMATE;      
      }

      // Blink duration has elapsed - turn off all LEDs and go to deep sleep state
      if (ts - blinkTs > BLINK_DURATION) {
        setLedStates (LOW, true);
        state = STATE_DEEPSLEEP;
      }
      
      break;
  

    /*
     * Deep Sleep State
     * Power down the CPU and wait for the next watchdog interrupt
     * Increment the seconds counter after every wake-up
     */
    case STATE_DEEPSLEEP:
    
      // Send the CPU into deep sleep, will come back within 1 second
      deepSleep();
      
      // Increment elapsed seconds upon wake-up 
      G.secondsElapsed++;

      // Detect long power button press to power down the system
      if (digitalRead (POWER_BUTTON_PIN) == LOW) {
        buttonPressSecs++;
        if (buttonPressSecs >= POWER_OFF_DELAY) {
          state = STATE_ANIMATE;
          break;
        }
      }
      else {
        buttonPressSecs = 0;                           
      }
      
      state = STATE_COUNTDOWN;
      
      break;

      
    /*
     * Animation State
     * Execute final LED anunation routine prior to shutdown
     */
    case STATE_ANIMATE: {
      uint16_t val, i;
      uint8_t lastIdx = G.animationIndex;
      /*
       * The animation phase will increase the current consumption which will result in a 
       * battery voltage drop. This may cause the CPU to hang and fail to power down the system
       * and subsequently drain the battery. This condition is referred-to as brownout.
       * The ATmega328P has a built-in brownout detection cricuit that shall initiate a system
       * reset as soon as the voltage drops below a certain threshold. The reset will ensure that 
       * the power transistor pin is no longer high, thus the system will be powered down. 
       * The following line enables the watchdog timer (WDT) as an additional measure, in case 
       * brownout detection has been deactivated by the fuse configuration. The WDT needs
       * to be peridically reset by the main loop, failing to do so will cause the WDT to
       * expire and subsequently perform a system reset. This ensures that the main loop needs
       * to be constantly running in order to keep the system powered on.
       * Enabling the WDT will disable the previously configured WDT interrupt, which is no
       * longer needed from this point onwards.
       */
      wdt_enable (WDTO_1S);
      
      /* 
       * The following code generates a random value that will be used as the index of the 
       * animation sequence to be executed.
       * The random noise from a floating (disconnected) analog input pin is used for generating 
       * the random seed for the arduino random() function. The ADC iis shortly powered on, then 
       * the result of several consecutive analog reads is accumulated and the ADC is disabled
       * again in order to reduce power consumption.
       */
      power_adc_enable ();
      for (i = 0; i < 20; i++) val += analogRead (RANDOM_SEED_APIN);
      power_adc_disable ();
      randomSeed (val);
      // Ensure that a different index is chosen every run
      // Randomize by repeatedly calling the random() function
      for (i = 0; i < val % 16 || G.animationIndex == lastIdx; i++) {
        G.animationIndex = random (NUM_ANIM);
      }
      
      state = STATE_ANIMATE_B;
   }
   case STATE_ANIMATE_B:
    
      // Call the animation routine
      rv = (*G.animate[G.animationIndex]) ();
      
      // The animation routine returns true upon finish
      if (rv) {
        setLedStates (LOW, true);
        state = STATE_SHUTDOWN;
      }

      // Reset the watchdog timer
      wdt_reset ();

      break;


    /*
     * Shutdown State
     * Turn the power off
     */
    case STATE_SHUTDOWN:

      // Remember the index of the last played animation
      EEPROM.write (EEPROM_ADDR, G.animationIndex);

      // Turn off the power supply
      digitalWrite (POWER_MOSFET_PIN, LOW);
      
      // Stay here until power is off
      while (1);       
      
      break;
    
  }
  /* End main state machine */

}
/* End Arduino main loop */



/*
 * Send the CPU Into Light Sleep Mode
 * 
 * The CPU will be waken-up within 1 millisecond by the Timer 0 millis() interrupt.
 * 
 * Please refer to ATmega328P datasheet Section 14.2. "Sleep Modes" for more 
 * information about the different sleep modes.
 */
void lightSleep () {
  
  set_sleep_mode (SLEEP_MODE_IDLE);  // Configure lowest sleep mode that keeps clk_IO for Timer 0
  sleep_enable ();                   // Prepare for sleep
  sleep_cpu ();                      // Send the CPU into seelp mode
  sleep_disable ();                  // CPU will wake-up here
}



/*
 * Send the CPU Into Deep Sleep Mode
 * 
 * The CPU will be waken-up within 1 second by the watchdog interrupt.
 * 
 * Please refer to ATmega328P datasheet Section 14.2. "Sleep Modes" for more
 * information about the different sleep modes.
 */
void deepSleep () {
  
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  // Configure lowest possible sleep mode
  cli ();                                // Disable interrupts (recommended for the BOD disable step)
  sleep_enable ();                       // Prepare for sleep
  sleep_bod_disable ();                  // Disable brown-out detection (BOD) (not possible in SLEEP_IDLE_MODE)
  sei ();                                // Enable interrupts
  sleep_cpu ();                          // Send the CPU into seelp mode
  sleep_disable ();                      // CPU will wake-up here
}



/*
 * Watchdog Interrupt Service Routine
 */
ISR (WDT_vect)  {
  // Do nothing
}



/*
 * LED Multiplexing Routine
 * 
 * Since all the LEDs share one common dropper resistor, they should not be truned on simultaneously 
 * and must be driven via multiplexing. This means that the LEDs are sequentially activated fast 
 * enough to create the illusion for the human eye that they are simultanously lit. 
 * This way we save ourselves 8 dropper resistors.
 * 
 * This function is called once per millisecond by the Arduino main loop, this results in a LED
 * power on duration of 1ms with a period of 9ms and a duty cycle of 1/9% for a 9 LED setup.
 */
void muxLeds () {
  static uint8_t idx = 0;

  digitalWrite (G.ledPin[idx], LOW);
  idx ++;
  if (idx >= NUM_LEDS) idx = 0;
  digitalWrite (G.ledPin[idx], G.ledState[idx]);
}



/*
 * Turn All LEDs On or Off
 * 
 * state: [HIGH|LOW]
 * apply: apply state immediately when set to true,
 *        otherwise wait for the multiplexing routine to do the job
 */
void setLedStates ( uint8_t state, bool apply ) {

  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    G.ledState[i] = state;
    if (apply) digitalWrite (G.ledPin[i], state);   
  }
}



/*
 * LED Animation Sequence 1
 * 
 * Returns true when animation sequence is finished
 */
bool animate1 () {
  static uint32_t blinkTs = 0;
  static uint8_t count = 0;
  static uint8_t idx = 0;
  uint32_t ts = millis ();

  if (ts - blinkTs > 100){ 
    
    if      (idx ==  0) G.ledState[5] = HIGH;
    else if (idx ==  1) G.ledState[2] = HIGH;
    else if (idx ==  2) G.ledState[3] = HIGH;
    else if (idx ==  3) G.ledState[7] = HIGH;
    else if (idx ==  4) G.ledState[1] = HIGH;
    else if (idx ==  5) G.ledState[6] = HIGH;
    else if (idx ==  6) G.ledState[0] = HIGH;
    else if (idx ==  7) G.ledState[4] = HIGH;
    else if (idx ==  8) G.ledState[8] = HIGH;
    else if (idx == 12) setLedStates (LOW, true);
    idx++;
    if (idx > 15) idx = 0;
    count++;
    blinkTs = ts;
  }

  if (count > 100 && idx == 0) return true;
  else                         return false;
}



/*
 * LED Animation Sequence 2
 * 
 * Returns true when animation sequence is finished
 */
bool animate2 () {
  static uint32_t blinkTs = 0;
  static uint8_t count = 0;
  static uint8_t idx = 0;
  uint32_t ts = millis ();

  if (ts - blinkTs > 100){
    setLedStates (LOW, true);
    if      (idx == 0) G.ledState[1] = G.ledState[3] = HIGH;
    else if (idx == 1) G.ledState[3] = G.ledState[5] = HIGH;
    else if (idx == 2) G.ledState[5] = G.ledState[6] = HIGH;
    else if (idx == 3) G.ledState[6] = G.ledState[8] = HIGH;
    else if (idx == 4) G.ledState[8] = G.ledState[7] = HIGH;
    else if (idx == 5) G.ledState[7] = G.ledState[6] = HIGH;
    else if (idx == 6) G.ledState[6] = G.ledState[4] = HIGH;
    else if (idx == 7) G.ledState[4] = G.ledState[2] = HIGH;
    else if (idx == 8) G.ledState[2] = G.ledState[0] = HIGH;
    else if (idx == 9) G.ledState[0] = G.ledState[1] = HIGH;
    idx++;
    if (idx > 9) idx = 0;
    count++;
    blinkTs = ts;
  }

  if (count > 100 && idx == 0) return true;
  else                         return false;
}



/*
 * LED Animation Sequence 3
 * 
 * Returns true when animation sequence is finished
 */
bool animate3 () {
  static uint32_t blinkTs = 0;
  static uint8_t count = 0;
  static uint8_t idx = 0;
  uint32_t ts = millis ();

  if (ts - blinkTs > 100){ 
    if      (idx == 0) G.ledState[0] = G.ledState[1] = HIGH;
    else if (idx == 1) G.ledState[2] = G.ledState[3] = HIGH;
    else if (idx == 2) G.ledState[4] = G.ledState[5] = HIGH;
    else if (idx == 3) G.ledState[6] = HIGH;
    else if (idx == 4) G.ledState[7] = G.ledState[8] = HIGH;
    else if (idx >= 8) setLedStates (LOW, true);
    idx++;
    if (idx > 11) idx = 0;
    count++;
    blinkTs = ts;
  }

  if (count > 100 && idx == 0) return true;
  else                         return false;
}



/*
 * LED Animation Sequence 4
 * 
 * Returns true when animation sequence is finished
 */
bool animate4 () {
  static uint32_t blinkTs = 0;
  static uint8_t count = 0;
  static uint8_t idx = 0;
  uint32_t ts = millis ();

  if (ts - blinkTs > 100){ 
    if      (idx == 0) G.ledState[7] = HIGH;
    else if (idx == 1) G.ledState[6] = HIGH;
    else if (idx == 2) G.ledState[4] = HIGH;
    else if (idx == 3) G.ledState[2] = HIGH;
    else if (idx == 4) G.ledState[0] = HIGH;
    else if (idx == 5) G.ledState[1] = HIGH;
    else if (idx == 6) G.ledState[3] = HIGH;
    else if (idx == 7) G.ledState[5] = HIGH;
    else if (idx == 8) G.ledState[8] = HIGH;
    else if (idx >= 12) setLedStates (LOW, true);
    idx++;
    if (idx > 15) idx = 0;
    count++;
    blinkTs = ts;
  }

  if (count > 100 && idx == 0) return true;
  else                         return false;
}



/*
 * LED Animation Sequence 5
 * 
 * Returns true when animation sequence is finished
 */
bool animate5 () {
  static uint32_t blinkTs = 0;
  static uint8_t count = 0;
  static uint8_t idx = 0;
  uint32_t ts = millis ();

  if (ts - blinkTs > 100){
    setLedStates (LOW, true);
    if      (idx == 0) G.ledState[0] = G.ledState[1] = HIGH;
    else if (idx == 1) G.ledState[0] = G.ledState[1] = G.ledState[2] = G.ledState[3] = HIGH;
    else if (idx == 2) G.ledState[2] = G.ledState[3] = G.ledState[4] = G.ledState[5] = HIGH;
    else if (idx == 3) G.ledState[4] = G.ledState[5] = G.ledState[6] = HIGH;
    else if (idx == 4) G.ledState[6] = G.ledState[7] = G.ledState[8] = HIGH;
    else if (idx == 5) G.ledState[7] = G.ledState[8] = HIGH;
    else if (idx == 6) ;
    idx++;
    if (idx > 6) idx = 0;
    count++;
    blinkTs = ts;
  }

  if (count > 100 && idx == 0) return true;
  else                         return false;
}
