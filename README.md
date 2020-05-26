# Example code using `iol.py` module

**Note: This document is currently being created and so is not yet suitable for reference.**

### Quick summary

* Examples inspired by Udemy ARM Cortex course: *"Embedded Systems Bare-Metal Programming Ground Up™ (STM32)"* — see references below

* Draws on low-level I/O class module (iol.py) for "bare-metal" access to STM32 memory locations and device registers — see references below

* Created on a Pyboard (PYB) v1.1 (with STM32F405RGT6 microcontroller) but should be readily adaptable to other Micropython systems

### History

I created the `iol.py` module when I wanted to experiment with "bare metal" access to STM32 device registers on a Pyboard v1.1.  I had obtained the Udemy course [*"Embedded Systems Bare-Metal Programming Ground Up™ (STM32)"*](https://www.udemy.com/course/embedded-systems-bare-metal-programming/) but was unable to use the examples directly as I didn't have a Windows PC on which to run the required Keil uVision environment.  I therefore chose to create classes to simulate "bare metal" access in Micropython rather than C.

I wasn't sure whether the Micropython system footprint would get in the way of "bare metal" accesses to device registers, but I didn't find this generally to be the case.  In most cases, I have been able to work around issues of contention by reading carefully the Micropython documentation and avoiding a few on-chip peripheral functions that are dedicated to Micropython.

Inevitably, code written in Micropython will run considerably slower than C code.  I only found this to be a problem, though, during one experiment with the SPI bus, in which Micropython code is unable to keep up with the high speed of the bus (up to 21 MHz).  Even in this case, I was able to work around this limitation by re-writing the critical section of code using inline assembler code.

The master repository for my `iol.py` module (included here as a sub-module) can be found [here](https://github.com/Chapmip/micropython-stm32-iol).

### Getting started

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

### Adding external components to Pyboard

I was able to add all of the necessary external components for my experiments by connecting a breadboard to the Pyboard through jumper wires (both readily obtainable from various suppliers for experiments with Pyboards, Arduino controllers etc), as per the photo below:

![Pyboard v1.1 with external components on breadboard](/photos/Pyboard%20v1.1%20with%20external%20components%20on%20breadboard.png?raw=true "Pyboard v1.1 with external components on breadboard")

### Example code

* [`clocks.py`](/clocks.py)
* [`c3.py`](/c3.py)
* [`c4.py`](/c4.py)
* [`c5.py`](/c5.py)
* [`c6.py`](/c6.py)
* [`c7.py`](/c7.py)
* [`c9.py`](/c9.py)
* [`c10.py`](/c10.py)

### References

* [Udemy course: *"Embedded Systems Bare-Metal Programming Ground Up™ (STM32)"*](https://www.udemy.com/course/embedded-systems-bare-metal-programming/)
* [STM32F405 Data Sheet](https://www.st.com/resource/en/datasheet/dm00037051.pdf)
* [STM32F405 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)
* [STM32 Cortex®-M4 Programming Manual](https://www.st.com/resource/en/programming_manual/dm00046982-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf)
* [MicroPython documentation](https://docs.micropython.org/en/latest/)
* [Pyboard v1.1 documentation](https://docs.micropython.org/en/latest/pyboard/quickref.html)
* [Serial Term for Chrome OS (by Ganehag)](https://chrome.google.com/webstore/detail/serial-term/fnjkimblohniildfepjhejeppenokhie)
* [Master repository for `iol.py` module](https://github.com/Chapmip/micropython-stm32-iol)
