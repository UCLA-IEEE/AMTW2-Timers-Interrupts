# AMTW2-Timers-Interrupts

The code for the Advanced Microcontroller Topics Workshop #2: Timers and Interrupts

This project contains a simple program for the TM4C123GH6PM microcontroller that will blink the onboard RGB led. No 3rd party libraries are used to control the GPIO; instead, to illustrate the operation of the memory-mapped peripheral registers, direct access on these registers is employed to control the GPIO. The program also implements its own delay function, which puts the processor into a busy loop for a fixed number of iterations.

The presentation slides that accompany this code can be viewed [here](https://docs.google.com/presentation/d/1K8AJamLNOSlNlZSXay_GrmVp_HXa3WnC28B-guS47_U/edit?usp=sharing).
