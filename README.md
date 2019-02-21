# Very Low Power Kids Toothbrush Timer

This Arduino sketch that implements a funny "little fish" kids toothbrush timer.

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

