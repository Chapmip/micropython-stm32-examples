#!/usr/bin/env micropython
#
# Copyright (c) 2020, Ian Chapman (Chapmip Consultancy)
#
# This software is licensed under the MIT license (see LICENSE.TXT)
#
# Examples inspired by Section 3 (GPIO) of Udemy ARM Cortex course:
# "Embedded Systems Bare-Metal Programming Ground Up[tm] (STM32)" --
# https://www.udemy.com/course/embedded-systems-bare-metal-programming/
#
# Draws on low-level I/O class module (iol.py) for "bare-metal" access
# to STM32 memory locations and device registers
#
# Created on a Pyboard (PYB) v1.1 (with STM32F405RGT6 microcontroller)
# but should be readily adaptable to other Micropython systems
#
# No external hardware requirements: only uses in-built red LED and
# user button on Pyboard
#
# See README.md for further information


from micropython import const
import pyb
import iol


# Port bit definitions

PA_RED = const(13)
PB_SW = const(3)


# Register definitions

ahb1enr = iol.Reg("RCC.AHB1ENR")

pa_moder = iol.RegArr("GPIOA.MODER", 16, 2)
pa_odr = iol.Reg("GPIOA.ODR")
pa_set = iol.Reg("GPIOA.BSRRL")     # Note: BSRRL *sets* output!
pa_clr = iol.Reg("GPIOA.BSRRH")     # Note: BSRRH *resets* output!

pb_moder = iol.RegArr("GPIOB.MODER", 16, 2)
pb_idr = iol.Reg("GPIOB.IDR")


# Basic flash example

def flash_basic():
    ahb1enr[0] = 1                  # Port A clock
    pa_moder[PA_RED] = 1            # Output

    for _ in range(5):
        pa_odr[PA_RED] = 1
        pyb.delay(500)
        pa_odr[PA_RED] = 0
        pyb.delay(500)


# Simpler flash example

def flash_simpler():
    ahb1enr[0] = 1                  # Port A clock
    pa_moder[PA_RED] = 1            # Output
    pa_odr[PA_RED] = 0

    for _ in range(10):
        pa_odr[PA_RED] ^= 1
        pyb.delay(500)


# BSRR flash example

def flash_bsrr():
    ahb1enr[0] = 1                  # Port A clock
    pa_moder[PA_RED] = 1            # Output

    for _ in range(5):
        pa_set[PA_RED] = 1
        pyb.delay(500)
        pa_clr[PA_RED] = 1
        pyb.delay(500)


# Button determines LED state

def led_button():
    ahb1enr[1:0] = 3                # Port A and Port B clocks
    pa_moder[PA_RED] = 1            # Output
    pb_moder[PB_SW] = 0             # Input

    while True:
        pa_odr[PA_RED] = pb_idr[PB_SW] ^ 1
