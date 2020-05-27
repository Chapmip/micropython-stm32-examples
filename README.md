# Example code using `iol.py` module

**Note: This document is currently being created and so is not yet suitable for reference.**

# Quick summary

* Examples inspired by Udemy ARM Cortex course: *"Embedded Systems Bare-Metal Programming Ground Up™ (STM32)"* — see references below

* Draws on low-level I/O class module (iol.py) for "bare-metal" access to STM32 memory locations and peripheral registers — see references below

* Created on a Pyboard (PYB) v1.1 (with STM32F405RGT6 microcontroller) but should be readily adaptable to other Micropython systems

# History

I created the `iol.py` module when I wanted to experiment with "bare metal" access to STM32 peripheral registers on a Pyboard v1.1.  I had obtained the Udemy course [*"Embedded Systems Bare-Metal Programming Ground Up™ (STM32)"*](https://www.udemy.com/course/embedded-systems-bare-metal-programming/) but was unable to use the examples directly as I didn't have a Windows PC on which to run the required Keil uVision environment.  I therefore chose to create classes to simulate "bare metal" access in Micropython rather than C.

I wasn't sure whether the Micropython system footprint would get in the way of "bare metal" accesses to peripheral registers, but I didn't find this generally to be the case.  In most cases, I have been able to work around issues of contention by reading carefully the Micropython documentation and avoiding a few on-chip peripheral functions that are dedicated to Micropython.

Inevitably, code written in Micropython will run considerably slower than C code.  I only found this to be a problem, though, during one experiment with the SPI bus, in which Micropython code is unable to keep up with the high speed of the bus (up to 21 MHz).  Even in this case, I was able to work around this limitation by re-writing the critical section of code using inline assembler code.

The master repository for my `iol.py` module (included here as a sub-module) can be found [here](https://github.com/Chapmip/micropython-stm32-iol).

# Getting started

When plugged into the USB port of a computer (using a Micro-USB B to USB A lead), the Pyboard is configured to appear as  both:

* An **MSC (Mass Storage Class) device** ⁠— for mounting into the filesystem of the computer as a dedicated drive showing the files in the flash memory of the Pyboard (either its inbuilt flash or an installed MicroSD card)

* A **VCP (Virtual Com Port) device** ⁠— for access via a serial terminal program on the computer to the REPL (Read–Evaluate–Print-Loop) environment on the Pyboard

On the Chromebook, I found that the Pyboard filesystem appeared automatically in the "Files" application a few seconds after inserting the USB connector.  I was then able to access the REPL prompt by installing the Chrome OS [Serial Term](https://chrome.google.com/webstore/detail/serial-term/fnjkimblohniildfepjhejeppenokhie) app and configuring the settings as per the screenshot below:

![Chrome OS Serial Terminal settings for Pyboard](/photos/Chrome%20OS%20Serial%20Terminal%20settings%20for%20Pyboard.png?raw=true "Chrome OS Serial Terminal settings for Pyboard")

To add the example code modules to the Pyboard, copy them (together with the `iol.py` module) from the computer into the top folder of the Pyboard filesystem, along with the .  From the REPL environment, the example modules can then be brought into operation by issuing an `import cX` instruction (where X=course number), as illustrated below:

![Running example code via Chrome OS Serial Terminal](/photos/Running%20example%20code%20via%20Chrome%20OS%20Serial%20Terminal.png?raw=true "Running example code via Chrome OS Serial Terminal")

**Note:** To avoid possible corruption of the Pyboard filesystem, it is important that the "Eject" operation is performed in the Chrome OS "Files" app before either:

* Disconnecting the USB cable; or
* Pressing the reset button on the Pyboard

In general, the reset button should be used as a last resort for a "cold start" of the Pyboard ⁠— for example, if the REPL environment has crashed.  In most other cases, it is adequate to perform a "warm start" by issuing a `<CONTROL-D>` on a blank line of the REPL prompt.

# Adding external components to Pyboard

I was able to add all of the necessary external components for my experiments by connecting a breadboard to the Pyboard through jumper wires (both readily obtainable from various suppliers for experiments with Pyboards, Arduino controllers etc), as per the photo below:

![Pyboard v1.1 with external components on breadboard](/photos/Pyboard%20v1.1%20with%20external%20components%20on%20breadboard.png?raw=true "Pyboard v1.1 with external components on breadboard")

# Example modules and external methods

The following modules and methods are offered as immediately usable (on a Pyboard v1.1) examples of "bare-metal" access to STM32 memory locations and peripheral registers using the `iol.py` module.  The module numbering follows the relevant sections of the Udemy ARM Cortex course.

## [`clocks.py`](/clocks.py) — calculate Pyboard clock frequencies from first principles

Calculate the Pyboard clock and bus frequencies from the external crystal value and the STM32 device register configuration settings, then compare these with the values reported by the system

To run this package (occurs automatically on `import`):

* Invoke a "warm start" on the Pyboard by issuing a `<CONTROL-D>` on a blank line of the REPL prompt
* Run the package automatically with `import clocks`

### External hardware requirements

None

## [`c3.py`](/c3.py) — examples inspired by Section 3 (GPIO) of Udemy ARM Cortex course

Before running each of the standalone examples in this package:

* Invoke a "warm start" on the Pyboard by issuing a `<CONTROL-D>` on a blank line of the REPL prompt
* Bring in the package with `import c3`

### External hardware requirements

None ⁠— only uses in-built red LED and user button on Pyboard

### `c3.flash_basic()`

Flash the red LED on and off five times by writing the ODR bit high then low

### `c3.flash_simpler()`

Flash the red LED on and off five times by toggling the ODR bit (exclusive-or)

### `c3.flash_bsrr()`

Flash the red LED on and off five times using the BSRR registers to set and reset the I/O port bit

`c3.led_button()`

Reflect the state of the user button on the red LED (pressed = on) in a continuous loop (interrupt with `<CONTROL-C>`)

## [`c4.py`](/c4.py) — examples inspired by Section 4 (USART) of Udemy ARM Cortex course

Before running each of the standalone examples in this package:

* Invoke a "warm start" on the Pyboard by issuing a `<CONTROL-D>` on a blank line of the REPL prompt
* Bring in the package with `import c4`

### External hardware requirement

For `echo_test()` and `rx_test()` — "loop-back" connection from X1 (PA0) to X2 (PA1) via series resistor (e.g. 1k) or jumper wire

### `c4.echo_test()`

Confirm the "loop-back" connection by toggling PA0 and displaying the state of PA1 each time

### `c4.tx_test()`

Send 5000 serial characters at 4800 baud over PA0, measuring the time taken (plus Micropython overheads) to confirm that the baud rate is in the expected range

### `c4.rx_test()`

Send 24 lines of alphanumeric characters serially at 4800 baud over PA0, displaying the serial characters received on PA1 (should echo the transmission)

## [`c5.py`](/c5.py) — examples inspired by Section 5 (Timers) of Udemy ARM Cortex course

Before running each of the standalone examples in this package:

* Invoke a "warm start" on the Pyboard by issuing a `<CONTROL-D>` on a blank line of the REPL prompt
* Bring in the package with `import c5`

### External hardware requirements

For `timer_capture()` only — LED via series resistor (e.g. 330 ohm) to ground from X1 (PA0) and "loop-back" connection from X1 to Y1 (PC6) via series resistor (e.g. 1k) or jumper wire

### 'c5.timer_basic()`

Set up Timer 2 to expire at 1.0 second intervals, toggling the state of the yellow LED manually when this happens, and measuring the total time taken for ten cycles

### `c5.timer_compare()'

Set up Timer 2 to run autonomously with a 1.0 second output compare period, with the compare match toggling the state of the yellow LED automatically — note this this will continue to occur even after the example has finished running!

### 'c5.timer_capture()'

Set up Timer 2 to run autonomously with a 1.0 second output compare period, with the compare match toggling the state of the external PA0 output automatically, and Timer 3 to run in input capture mode on the linked external input PC6, measuring the times at which changes occur and displaying the intervals

### `c5.timer_pwm()`

Sets up Timer 2 to provide a PWM output to the external PA0 output, varying the brightness of the external LED according to the inputted percentage value

## [`c6.py`](/c6.py) — examples inspired by Section 6 (Interrupts) of Udemy ARM Cortex course

*Note: It is not readily possible to create "bare-metal" interrupt handlers in Micropython, so these examples either use the Micropython "callback" mechanism when available (external interrupt and timer) or just report the way that interrupts are set up by Micropython (USART)*

Before running each of the standalone examples in this package:

* Invoke a "warm start" on the Pyboard by issuing a `<CONTROL-D>` on a blank line of the REPL prompt
* Bring in the package with `import c6`

### External hardware requirement

For `int_external()` and `int_timer()` — LED via series resistor (e.g. 330 ohm) to ground from X1 (PA0) and "loop-back" connection from X1 to Y1 (PC6) via series resistor (e.g. 1k) or jumper wire

### `c6.int_external()`

Set up an interrupt callback handler (in Micropython) for high-to-low transitions on external input pin PC6, then control the state of PA0 in a loop according to the inputted 1/0 value, reporting the interrupt activity

### `c6.int_timer()`

Set up an interrupt callback handler (in Micropython) for Timer 2 expiry, then display the updated interrupt count when the user inputs to request this

### `c6.int_uart()`

Set up the UART 4 in Micropython, then prompt for activity over the serial port and report on the states of the STM32 peripheral registers associated with the UART (note: interrupt callback handlers are not available for UARTs in the STM32 version of Micropython)

## [`c7.py`](/c7.py) — examples inspired by Section 7 (ADC) of Udemy ARM Cortex course

Before running each of the standalone examples in this package:

* Invoke a "warm start" on the Pyboard by issuing a `<CONTROL-D>` on a blank line of the REPL prompt
* Bring in the package with `import c7`

### External hardware requirement

For `adc_basic()` only — 50k potentiometer (10k or 100k also okay) from +3.3V supply to ground, with wiper connected to X7 (PA6) via series resistor (e.g. 330 ohm) or jumper wire

### `c7.adc_basic()`

Set up ADC1 to read analogue values from the potentiometer on PA6 in a loop, converting the readings into percentage values calculated according to the configured ADC resolution (12 bits by default)

### `c7.adc_temp()`

Set up ADC1 to read the internal temperature reading from the CPU, then prompt a series of 10 readings in a loop at 1.0 second intervals, displaying the results converted to Celsius

### `c7.adc_auto()`

Set up ADC1 to read the internal temperature reading from the CPU, then set up Timer 2 to trigger these readings automatically at 3.0 second intervals, displaying the results of 10 readings in a loop, converted to Celsius

## There is no `c8.py`!

Section 8 of the Udemy ARM Cortex course requires an external Liquid Crystal Display (LCD), which I did not have.

## [`c9.py`](/c9.py) — examples inspired by Section 9 (I2C) of Udemy ARM Cortex course

Before running each of the standalone examples in this package:

* Invoke a "warm start" on the Pyboard by issuing a `<CONTROL-D>` on a blank line of the REPL prompt
* Bring in the package with `import c9`

### External hardware requirements

For `i2c_test_ext()` only — Bosch BMP280 digital temperature and pressure sensor powered by +3.3V supply (VDD/VDDIO and GND) with CSB connected via a 10k pull-up resistor to +3.3V to select I2C bus mode, SCL/SCK connected to X9, SDA/SDI connected to X10, and SDO left open

### `c9.i2c_test_pyb()`

Test the inbuilt accelerometer on the Pyboard using Micropython (pyb) I2C calls — NOT bare-metal methods!

*Note: the accelerometer does not respond at all until powered from the PB5 output line, and then does not return valid readings until configured correctly*

### `c9.i2c_test_int()`

Read from the internal I2C accelerometer on the Pyboard using a stack of bare-metal methods addressing the STM32 I2C1 peripheral block

### `c9.i2c_scan()`

Scan for any active I2C devices on the bus using a stack of bare-metal methods addressing the STM32 I2C1 peripheral block, reporting the addresses polled and any acknowledgements received

### `c9.i2c_test_ext()`

Read from the external Bosch BMP280 digital temperature and pressure sensor over the I2C bus using a stack of bare-metal methods addressing the STM32 I2C1 peripheral block

*Note: the calculation of temperature and pressure from the raw readings is a complex computation that needs to take into account the calibration values that are pre-programmed into the device, so this procedure is not implemented and raw readings only are returned*

## [`c10.py`](/c10.py) — example inspired by Section 10 (SPI) of Udemy ARM Cortex course

*Note: This example exposes the limitations of "bare-metal" routines running on top of Micropython, as the SPI transmit/receive functions are unable to keep up with the high speed of the SPI bus, even at its slowest setting (160 KHz on the Pyboard), as flagged by the receiver overrun errors.  However, a workaround has been achieved using inline assembler code for the time-critical part of these functions to yield an execution speed that is able to keep up with the SPI bus even at its maximum speed (21 MHz on the Pyboard).*

Before running the standalone example in this package:

* Invoke a "warm start" on the Pyboard by issuing a `<CONTROL-D>` on a blank line of the REPL prompt
* Bring in the package with `import c10`

### External hardware requirements

For this example — Bosch BMP280 digital temperature and pressure sensor powered by +3.3V supply (VDD/VDDIO and GND) with CSB connected to Y5 and *also* via a 10k pull-down resistor to ground to select SPI bus mode, SCL/SCK connected to Y6, SDO connected to Y7 and SDA/SDI connected to Y8.

Use short connection wires (e.g. 10cm), especially if running the SPI bus close to its maximum speed!

### `c10.spi_test_ext()`

Read from the external Bosch BMP280 digital temperature and pressure sensor over the SPI bus using a stack of bare-metal methods addressing the STM32 SPI2 peripheral block

*Note: the calculation of temperature and pressure from the raw readings is a complex computation that needs to take into account the calibration values that are pre-programmed into the device, so this procedure is not implemented and raw readings only are returned*

### Time-critical section

The inline assembly code method `spi_tx_rx_asm()` contains the speed-critical element for handling the SPI bus in real time.  Note that receiver overrun errors may still occur if a Micropython system interrupt occurs during its execution.  This could be avoided by disabling interrupts at the start of this function and re-enabling them afterwards, but at the cost of introducing undesirable extra latency into the rest of the system.  If this approach is chosen, then it may be prudent to run the SPI bus at the highest possible speed (consistent with the hardware capabilities) so that the latency introduced by temporary disabling of interrupts is minimised.

# References

* [Udemy course: *"Embedded Systems Bare-Metal Programming Ground Up™ (STM32)"*](https://www.udemy.com/course/embedded-systems-bare-metal-programming/)
* [STM32F405 Data Sheet](https://www.st.com/resource/en/datasheet/dm00037051.pdf)
* [STM32F405 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)
* [STM32 Cortex®-M4 Programming Manual](https://www.st.com/resource/en/programming_manual/dm00046982-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf)
* [MicroPython documentation](https://docs.micropython.org/en/latest/)
* [Pyboard v1.1 documentation](https://docs.micropython.org/en/latest/pyboard/quickref.html)
* [Serial Term for Chrome OS (by Ganehag)](https://chrome.google.com/webstore/detail/serial-term/fnjkimblohniildfepjhejeppenokhie)
* [Master repository for `iol.py` module](https://github.com/Chapmip/micropython-stm32-iol)
