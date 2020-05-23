# Work in progress...

**This document is currently being created and so is not yet suitable for reference.**

### General description of example code

* Examples inspired by Udemy ARM Cortex course: *"Embedded Systems Bare-Metal Programming Ground Up™ (STM32)"* --- see references below.

* Draws on low-level I/O class module (iol.py) for "bare-metal" access to STM32 memory locations and device registers --- see references below.

* Created on a Pyboard (PYB) v1.1 (with STM32F405RGT6 microcontroller) but should be readily adaptable to other Micropython systems.

### Photos to support example code

![Running example code via Chrome OS Serial Terminal](/photos/Running%20example%20code%20via%20Chrome%20OS%20Serial%20Terminal.png?raw=true "Running example code via Chrome OS Serial Terminal")

![Pyboard v1.1 with external components on breadboard](/photos/Pyboard%20v1.1%20with%20external%20components%20on%20breadboard.png?raw=true "Pyboard v1.1 with external components on breadboard")

### References

* [Udemy course: *"Embedded Systems Bare-Metal Programming Ground Up™ (STM32)"*](https://www.udemy.com/course/embedded-systems-bare-metal-programming/)
* [STM32F405 Data Sheet](https://www.st.com/resource/en/datasheet/dm00037051.pdf)
* [STM32F405 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)
* [STM32 Cortex®-M4 Programming Manual](https://www.st.com/resource/en/programming_manual/dm00046982-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf)
* [MicroPython documentation](https://docs.micropython.org/en/latest/)
* [Pyboard v1.1 documentation](https://docs.micropython.org/en/latest/pyboard/quickref.html)
* [Serial Term for Chrome OS (by Ganehag)](https://chrome.google.com/webstore/detail/serial-term/fnjkimblohniildfepjhejeppenokhie)
* [My `iol.py` module](https://github.com/Chapmip/micropython-stm32-iol)

### Example code

* [`clocks.py`](/clocks.py)
* [`c3.py`](/c3.py)
* [`c4.py`](/c4.py)
* [`c5.py`](/c5.py)
* [`c6.py`](/c6.py)
* [`c7.py`](/c7.py)
* [`c9.py`](/c9.py)
* [`c10.py`](/c10.py)
