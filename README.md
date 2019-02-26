# Very Low Power Kids Toothbrush Timer

<p align="center">
<img src="https://raw.githubusercontent.com/microfarad-de/brush-timer/master/doc/2019-02-25-211951.jpg" alt="drawing" width="400"/>
</p>

This Arduino sketch that implements a funny "little fish" kids toothbrush timer.

Unless stated otherwise within the source file headers, please feel free to use and distribute 
this code under the *GNU General Public License v3.0*.

## Theory of Operation

The device features 9 LEDs arranged in a "little fish" pattern. When turned on, 
the eyes of the fish will start to blink once per second. More and more LEDs begin to 
blink as time passes. One of few random animation sequences is activated as soon as the 
recommended two minute tooth brushing duration elapses, then the system will power 
itself off.

The device implements a soft power on circuit using a tactile switch and two 
FET transistors. Briefly pressing the power button will power up the system and initiate
the timer sequence. Pressing the power button during aprroximately 2 seconds while the  
device is on will skip the timer sequence and directly activate the LED animation 
sequence prior to powering down the system.

The device is designed to consume a very low current and is able to run on a single CR2025 
or similar 3V Lithium cell for thousands of cycles. The current consumption is around 100μA 
during the deep sleep phase and about 1.5mA when all the LEDs are on. The device consumes no 
current when the power is off.

In order to reduce the bill of material, all 9 LEDs share one common dropper resistor.
Turning on multiple LEDs simultaneously is not recommended as it will result in some of 
the LEDs glowing brighter than others. A multiplexing routine is used for sequentially 
turning on one LED at a time and doing this fast enough to create the illusion that they 
are simultaneously lit due the the persistance of the human vision.

This sketch has been implemented and tested on an ATMega328P based Arduino Pro Mini 
compatible board running on 3.3V/8MHz.

It is recommended to activate the watchdog support on the Arduino bootloader
by defining the WATCHDOG_MODS macro. This will reduce the bootloader's power-up 
delay, thus invalidating the need to hold the power button for around 2 seconds for 
the system to turn on.

## Circuit Diagram

You can find the circuit diagram for this device under the follwoing link:

https://github.com/microfarad-de/brush-timer/raw/master/doc/brush-timer-schematic.pdf

## What Can Be Improved

Power consumption could be further reduced by avoiding the use of SLEEP_MODE_IDLE and fully relying on SLEEP_MODE_PWR_DOWN instead. This will however disable Timer 0 thus the millis() function will no longer be usable. Consequently, all time measurements would have to be made using the watchdog timer interrupt which has a minimum configurable period of 16 milliseconds. In other words, the minimum measurable time duration will increase from 1ms to 16ms.

LED multiplexing is the only functionality that relies on the 1ms time duration, as each led is lit up during 1ms per multiplexing cycle. Disabling LED multiplexing would require the addition of a dedicated dropper resistor for each of the LEDs, or accepting the compromise of the LED intensity changing depending on how many LEDs are being lit.



