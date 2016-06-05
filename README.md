# AMTW2-Timers-Interrupts

The code for the Advanced Microcontroller Topics Workshop #2: Timers and Interrupts

This project contains a simple program for the TM4C123GH6PM microcontroller that will control a single hobby RC servo motor. No 3rd party libraries are used to control the GPIO or set up the timers; instead, to illustrate the operation of the memory-mapped peripheral registers, direct access on these registers is employed to control the registers.

The program first sets up the GPIO and a single 32-bit hardware timer in count-up mode. A single ISR firing on the timer timeout handles a state machine for the output PWM signal generation. Each time the ISR fires, it reloads the timeout value with either the next PWM high period, or the PWM period remainder. It also toggles the state of the output pin.

The extra credit assignment for this project is to extend the code to control more than one servo motor. _Hint: you still only need one timer! Consider how long each PWM period is vs. how long the signal will be high for at the absolute maximum._

The presentation slides that accompany this code can be viewed [here](https://docs.google.com/presentation/d/1K8AJamLNOSlNlZSXay_GrmVp_HXa3WnC28B-guS47_U/edit?usp=sharing).
