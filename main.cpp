/**
 * @author Kevin Balke
 * @brief Servo control example program for the TM4C123GH6PM microcontroller.
 * @note Bracketed references (e.g., [100], [10.5]) refer to the appropriate
 *           page or section in the TM4C123GH6PM datasheet:
 *           http://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf
 */

#include "debug_serial.h"
#include "driverlib/sysctl.h"

#include <stdbool.h>
#include <stdint.h>

#define dptr(x) (*reinterpret_cast<volatile uint32_t*>(x))

extern "C" {
    /**
     * Delays for a given number of loop cycles. Each loop takes 3 processor
     *     cycles to complete.
     */
    __attribute__((naked))
    void myDelay(uint32_t cycles)
    {
        __asm("subs R0, #1\n"
              "bne myDelay\n"
              "bx lr\n");
    }

    const static uint32_t minMatchVal = 80000; // Computed from F_CPU*0.001
    const static uint32_t maxMatchVal = 160000; // Computed from F_CPU*0.002
    const static uint32_t periodVal = 1600000; // Computed from F_CPU/(8*50)
    uint32_t timer0MatchBuffer = minMatchVal;

    void setInterval(uint32_t ival)
    {
        /*
         * Load the interval register [723].
         *
         * From the datasheet:
         * Base address for 16/32 bit Timer 0 module = 0x40030000
         * Address offset for register 10 (GPTMTAILR, General-purpose Timer
         *     Module Timer A Interval Load) = 0x028
         */
        dptr(0x40030000 + 0x028) = ival - 1;
    }

    uint32_t getInterval()
    {
        /*
         * Load the interval register [723].
         *
         * From the datasheet:
         * Base address for 16/32 bit Timer 0 module = 0x40030000
         * Address offset for register 10 (GPTMTAILR, General-purpose Timer
         *     Module Timer A Interval Load) = 0x028
         */
        return (dptr(0x40030000 + 0x028) + 1);
    }

    void Timer0Handler()
    {
        static bool pwm_state = true;

        uint32_t GPTMMIS_val = dptr(0x40030000 + 0x020);

        // Check if the Timer 0A time-out bit is set in the masked interrupt
        // status register [753].
        if(GPTMMIS_val & 0x00000001)
        {
            // Timer Time-Out Event
            if(pwm_state)
            {
                // Write the pin to high
                dptr(0x40025000 + (0x0E << 2)) = 0x02;
                setInterval(timer0MatchBuffer);
            }
            else
            {
                // Write the pin to low
                dptr(0x40025000 + (0x0E << 2)) = 0x00;
                setInterval(periodVal - getInterval());
            }

            // Switch the pwm_state
            pwm_state = !pwm_state;

            // Clear the time-out bit by writing to the Timer Interrupt Clear
            // register.
            dptr(0x40030000 + 0x024) |= 0x00000001;
        }
    }
}

int main(void)
{
    /*
     * Configure the system clock to operate at 80 MHz (400 MHz PLL / 5).
     * Drive the PLL from an external 16 MHz crystal.
     *
     * SYSCTL_SYSDIV_2_5 indicates to bypass the PLL predivider (400MHz), and
     *     divide by 5 into the System Clock.
     * SYSCTL_USE_PLL indicates to use the PLL instead of another source.
     * SYSCTL_XTAL_16MHZ indicates that an external crystal of 16MHz is to be
     *     sourced.
     * SYSCTL_OSC_MAIN indicates to use the main oscillator as the source for
     *     the PLL.
     *
     * (Don't worry about this!)
     */
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                   SYSCTL_OSC_MAIN);

    /*
     * Enable clocking to GPTM Timer 0 [340].
     *
     * From the datasheet:
     * Base address for the System Control module = 0x400FE00
     * Address offset for register 59 (RCGCTIMER, GPT Run Mode Clock Gating
     *     Control) = 0x604
     * Bit in the register for Timer 0: 0
     *
     * Operation should be:
     * dptr(0x400FE000 + 0x604) |= 0x00000001;
     */
    dptr(0x400FE000 + 0x604) |= 0x00000001;

    /*
     * Ensure that Timer 0 is disabled [722].
     *
     * From the datasheet:
     * Base address for 16/32 bit Timer 0 module = 0x40030000
     * Address offset for register 4 (GPTMCTL, General-purpose Timer Module
     *     Control) = 0x00C
     * Bits in the register for TnEN fields = (8,0)
     * Value we want (for 32-bit timer configuration) = (0,0)
     *
     * Operation should be:
     * dptr(0x40030000 + 0x00C) &= ~0x00000101;
     */
    dptr(0x40030000 + 0x00C) &= ~0x00000101;

    /*
     * Set Timer 0 to 32 bit (concatenated) mode [722].
     *
     * From the datasheet:
     * Base address for 16/32 bit Timer 0 module = 0x40030000
     * Address offset for register 1 (GPTMCFG, General-purpose Timer Module
     *     Configuration) = 0x000
     * Bits in the register for GPTMCFG field = [2:0]
     * Value we want (for 32-bit timer configuration) = 0x0
     *
     * Operation should be:
     * dptr(0x40030000 + 0x000) &= ~0x00000007;
     */
    dptr(0x40030000 + 0x000) &= ~0x00000007;

    /*
     * Set Timer 0 to periodic mode, count up [722].
     *
     * From the datasheet:
     * Base address for 16/32 bit Timer 0 module = 0x40030000
     * Address offset for register 2 (GPTMTAMR, General-purpose Timer Module
     *     Timer A Mode) = 0x004
     * Bits in the register for TAMR field = [1:0]
     * Value we want (for periodic mode) = 0x2
     *
     * Bit in the register for TACDIR field = 4
     * Value we want (for count up) = 0x1
     *
     * Operation should be:
     * dptr(0x40030000 + 0x004) |= 0x00000012;
     */
     dptr(0x40030000 + 0x004) |= 0x00000012;

    /*
     * Enable Timer 0 time-out interrupt [722].
     *
     * From the datasheet:
     * Base address for 16/32 bit Timer 0 module = 0x40030000
     * Address offset for register 2 (GPTMIMR, General-purpose Timer Module
     *     Interrupt Mask) = 0x018
     * Bit in the register for TATOIM field = 0
     *
     * Operation should be:
     * dptr(0x40030000 + 0x018) |= 0x00000001;
     */
     dptr(0x40030000 + 0x018) |= 0x00000001;

    /*
     * Enable interrupt 19 (Timer 0A interrupt) [104].
     *
     * From the datasheet:
     * Base address for NVIC (Nested-vectored Interrupt Controller) = 0xE000E000
     * Address offset for register 4 (EN0, Interrupt 0-31 Set Enable) = 0x100
     * Bit in the register for interrupt 19 = 19
     *
     * Operation should be:
     * dptr(0xE000E000 + 0x100) |= 0x00080000;
     */
    dptr(0xE000E000 + 0x100) |= 0x00080000;

    // Load the interval register
    setInterval(periodVal);

    /*
     * Enable clocking to GPIO port F [340].
     *
     * From the datasheet:
     * Base address for System Control module = 0x400FE000
     * Address offset for register 60 (RCGCGPIO, GPIO Run Mode Clock Gating
     *     Control) = 0x608
     * Bit in the register for port F = 5
     *
     * Operation should be:
     * dptr(0x400FE000 + 0x608) |= (1 << 5);
     *
     * Equivalent DriverLib call: SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
     */
    dptr(0x400FE608) |= 0x00000020;

    /*
     * Select the GPIO function for GPIO port F pins 1,2,3 (onboard R,G,B LEDs)
     *     [672, 10.5-R10].
     *
     * From the datasheet:
     * Base address for GPIO port F (APB) = 0x40025000
     * Address offset for register 10 (GPIOAFSEL, GPIO Alternate Function
     *     Select) = 0x420
     * Bits in the register for port F pins 1,2,3 = 1,2,3
     *
     * 0 = GPIO function
     * 1 = Alternate function
     *
     * Operation should be:
     * dptr(0x40025000 + 0x420) &= ~((1 << 1) | (1 << 2) | (1 << 3));
     */
    dptr(0x40025420) &= ~0x0000000E;

    /*
     * Select the 2-mA drive strength for GPIO port F pins 1,2,3 (onboard R,G,B
     *     LEDs) [673, 10.5-R11].
     *
     * From the datasheet:
     * Base address for GPIO port F (APB) = 0x40025000
     * Address offset for register 11 (GPIODR2R, GPIO 2-mA Drive Select) =
     *     0x500
     * Bits in the register for port F pins 1,2,3 = 1,2,3
     *
     * Operation should be:
     * dptr(0x40025000 + 0x500) |= ((1 << 1) | (1 << 2) | (1 << 3));
     */
    dptr(0x40025500) |= 0x0000000E;

    /*
     * Disable pull-up and pull-down resistors for GPIO port F pins 1,2,3
     *     (onboard R,G,B LEDs) [677, 10.5-R15], [679, 10.5-R16].
     *
     * From the datasheet:
     * Base address for GPIO port F (APB) = 0x40025000
     * Address offset for register 15 (GPIOPUR, GPIO Pull-Up Select) = 0x510
     * Address offset for register 16 (GPIOPDR, GPIO Pull-Down Select) = 0x514
     * Bits in the register for port F pins 1,2,3 = 1,2,3
     *
     * 0 = Resistor disabled
     * 1 = Resistor enabled
     *
     * Operations should be:
     * dptr(0x40025000 + 0x510) &= ~((1 << 1) | (1 << 2) | (1 << 3));
     * dptr(0x40025000 + 0x514) &= ~((1 << 1) | (1 << 2) | (1 << 3));
     */
    dptr(0x40025510) &= ~0x0000000E;
    dptr(0x40025514) &= ~0x0000000E;

    /*
     * Disable slew rate control for GPIO port F pins 1,2,3 (onboard R,G,B
     *     LEDs) [681, 10.5-R17].
     *
     * From the datasheet:
     * Base address for GPIO port F (APB) = 0x40025000
     * Address offset for register 17 (GPIOSLR, GPIO Slew Rate Control Select)
     *     = 0x518
     * Bits in the register for port F pins 1,2,3 = 1,2,3
     *
     * 0 = SRC disabled
     * 1 = SRC enabled
     *
     * Operation should be:
     * dptr(0x40025000 + 0x518) &= ~((1 << 1) | (1 << 2) | (1 << 3));
     */
    dptr(0x40025518) &= ~0x0000000E;

    /*
     * Enable the digital functions for GPIO port F pins 1,2,3 (onboard R,G,B
     *     LEDs) [682, 10.5-R18].
     *
     * From the datasheet:
     * Base address for GPIO port F (APB) = 0x40025000
     * Address offset for register 18 (GPIODEN, GPIO Digital Enable) = 0x51C
     * Bits in the register for port F pins 1,2,3 = 1,2,3
     *
     * Operation should be:
     * dptr(0x40025000 + 0x51C) |= ((1 << 1) | (1 << 2) | (1 << 3));
     */
    dptr(0x4002551C) |= 0x0000000E;

    /*
     * Equivalent DriverLib call for the preceeding GPIO configuration:
     *     GPIOPadConfigSet(GPIO_PORTF_BASE, 0x0F, GPIO_STRENGTH_2MA,
     *                      GPIO_PIN_TYPE_STD);
     */

    /*
     * Set pin direction for GPIO port F pins 1,2,3 (onboard R,G,B LEDs) to
     *     output [663, 10.5-R2].
     *
     * From the datasheet:
     * Base address for GPIO port F (APB) = 0x40025000
     * Address offset for register 2 (GPIODIR, GPIO Direction) = 0x400
     *                                                           ^^^^^
     *                                              (why is this 0x400?)
     * Bits in the register for port F pins 1,2,3 = 1,2,3
     *
     * Operation should be:
     * dptr(0x40025400) |= (1 << 1) | (1 << 2) | (1 << 3);
     *
     * Equivalent DriverLib call:
     *     GPIODirModeSet(GPIO_PORTF_BASE, 0x0F, GPIO_DIR_MODE_OUT);
     */
    dptr(0x40025400) |= 0x0000000E;

    /*
     * Enable Timer 0 [722].
     *
     * From the datasheet:
     * Base address for 16/32 bit Timer 0 module = 0x40030000
     * Address offset for register 4 (GPTMCTL, General-purpose Timer Module
     *     Control) = 0x00C
     * Bits in the register for TnEN fields = (8,0)
     * Value we want (for 32-bit timer configuration) = (1,1)
     *
     * Operation should be:
     * dptr(0x40030000 + 0x00C) |= 0x00000001;
     */
    dptr(0x40030000 + 0x00C) |= 0x00000001;

    while(1)
    {
        timer0MatchBuffer+=1000;
        if(timer0MatchBuffer >= maxMatchVal)
            timer0MatchBuffer = minMatchVal;

        myDelay(1000000ul);
    }

    return 0;
}
