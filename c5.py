#!/usr/bin/env micropython
#
# Copyright (c) 2020, Ian Chapman (Chapmip Consultancy)
#
# This software is licensed under the MIT license (see LICENSE.TXT)
#
# Examples inspired by Section 5 (Timers) of Udemy ARM Cortex course:
# "Embedded Systems Bare-Metal Programming Ground Up[tm] (STM32)" --
# https://www.udemy.com/course/embedded-systems-bare-metal-programming/
#
# Draws on low-level I/O class module (iol.py) for "bare-metal" access
# to STM32 memory locations and device registers
#
# Created on a Pyboard (PYB) v1.1 (with STM32F405RGT6 microcontroller)
# but should be readily adaptable to other Micropython systems
#
# External hardware requirements: LED via series resistor (e.g. 330 ohm)
# to ground from X1 (PA0) and "loop-back" connection from X1 to Y1 (PC6)
# via series resistor (e.g. 1k) or jumper wire
#
# See README.md for further information


from micropython import const
import utime
import pyb
import iol


# Get timer clock frequency in Hz
# Note that timer clock is doubled if bus prescaler factor != 1
# as per section 7.2 ("Clocks") of RM0090 Reference Manual

tmr_apb1_clk = pyb.freq()[2]            # Get pclk1 value from tuple
cfgr = iol.Reg("RCC.CFGR")              # Need PPRE1 field for APB1
if cfgr[12] == 1:                       # If bits [12:10] != 0xx ...
    tmr_apb1_clk *= 2                   # .. then adjust timer clock
del cfgr


# Timer parameters

PSC_CLK = const(5000)                   # Prescaler clock frequency
psc_val = tmr_apb1_clk // PSC_CLK       # Calculate prescaler value
tmr_sec = tmr_apb1_clk // psc_val       # Timer period for 1 second


# Port bit definitions

PA_RED = const(13)                      # Internal LEDs
PA_GRN = const(14)
PA_YEL = const(15)

PA_OUT = const(0)                       # External pin X1
PC_INP = const(6)                       # External pin Y1


# Register definitions

ahb1enr = iol.Reg("RCC.AHB1ENR")
apb1enr = iol.Reg("RCC.APB1ENR")

pa_moder = iol.RegArr("GPIOA.MODER", 16, 2)
pa_odr = iol.Reg("GPIOA.ODR", 16)
pa_afr = iol.RegArr("GPIOA.AFR0,AFR1", 8, 4)

pc_moder = iol.RegArr("GPIOC.MODER", 16, 2)
pc_odr = iol.Reg("GPIOC.ODR", 16)
pc_afr = iol.RegArr("GPIOC.AFR0,AFR1", 8, 4)

t2_psc = iol.Reg("TIM2.PSC", 16)
t2_arr = iol.Reg("TIM2.ARR")            # Full 32 bits for TIM2
t2_cnt = iol.Reg("TIM2.CNT")            # Full 32 bits for TIM2
t2_cr1 = iol.Reg("TIM2.CR1", 16)
t2_sr = iol.Reg("TIM2.SR", 16)

t2_ccmr = iol.RegArr("TIM2.CCMR1,CCMR2", 2, 8)
t2_ccer = iol.RegArr("TIM2.CCER", 4, 4)
t2_ccr1 = iol.Reg("TIM2.CCR1")          # Full 32 bits for TIM2

t3_psc = iol.Reg("TIM3.PSC", 16)
t3_arr = iol.Reg("TIM3.ARR", 16)        # Only 16 bits for TIM3
t3_cnt = iol.Reg("TIM3.CNT", 16)        # Only 16 bits for TIM3
t3_cr1 = iol.Reg("TIM3.CR1", 16)
t3_sr = iol.Reg("TIM3.SR", 16)

t3_ccmr = iol.RegArr("TIM3.CCMR1,CCMR2", 2, 8)
t3_ccer = iol.RegArr("TIM3.CCER", 4, 4)
t3_ccr1 = iol.Reg("TIM3.CCR1", 16)      # Only 16 bits for TIM3


# Timer capture/compare channels (indexes to CCMR/CCER arrays)

TIM_CC1 = const(0)
TIM_CC2 = const(1)
TIM_CC3 = const(2)
TIM_CC4 = const(3)


# Basic timer example

def timer_basic():
    ahb1enr[0] = 1                      # Port A clock
    pa_moder[PA_YEL] = 1                # Output
    pa_moder[PA_GRN] = 1                # Output
    pa_odr[PA_GRN] = 1                  # LED on

    apb1enr[0] = 1                      # Timer 2 clock
    t2_psc[:] = psc_val - 1             # Set prescaler
    t2_arr[:] = tmr_sec - 1             # 1.0s timer period
    t2_cnt[:] = 0                       # Dummy load

    pa_odr[PA_YEL] = 1                  # LED on

    start_time = utime.ticks_ms()       # Prepare to measure time

    t2_cr1[:] = 1                       # Enable timer
    t2_sr[0] = 0                        # Clear timer flag

    for n in range(10):
        while t2_sr[0] == 0:            # Wait for timer flag
            pass
        t2_sr[0] = 0                    # Clear timer flag
        pa_odr[PA_YEL] ^= 1             # Toggle LED

    finish_time = utime.ticks_ms()
    duration = utime.ticks_diff(finish_time, start_time) / 1000
    print("Duration:", duration, "secs")

    pa_odr[PA_YEL] = 0                  # LEDs off
    pa_odr[PA_GRN] = 0                  # LEDs off



# Automatic timer using compare

def timer_compare():
    ahb1enr[0] = 1                      # Port A clock
    pa_moder[PA_YEL] = 2                # Alternate function
    pa_moder[PA_GRN] = 1                # Output
    pa_afr[PA_YEL] = 1                  # AF1 mode for TIM2 CH1 on PA15
    pa_odr[PA_GRN] = 1                  # LED on

    apb1enr[0] = 1                      # Timer 2 clock
    t2_psc[:] = psc_val - 1             # Set prescaler
    t2_arr[:] = tmr_sec - 1             # 1.0s timer period

    t2_ccer[TIM_CC1] = 0                # Disable CC1 for write to CCMR
    t2_ccmr[TIM_CC1] = 0x30             # CC1 compare mode, toggle o/p
    t2_ccer[TIM_CC1] = 1                # CC1 - enable OC1, disable OC1N
    t2_ccr1[:] = 0                      # Compare match value

    t2_cnt[:] = 0                       # Dummy load
    t2_cr1[:] = 1                       # Enable timer

    pa_odr[PA_GRN] = 0                  # LED off


# Wait for Timer 3 capture and return value

def t3_capture():
    while t3_sr[1] == 0:                # Wait for capture flag
        pass
    return t3_ccr1[:]                   # Return capture value


# Compare and capture using linked external pins

def timer_capture():
    ahb1enr[0] = 1                      # Port A clock
    ahb1enr[2] = 1                      # Port C clock

    pa_moder[PA_OUT] = 2                # Alternate function
    pa_afr[PA_OUT] = 1                  # AF1 mode for TIM2 CH1 on PA0

    pc_moder[PC_INP] = 2                # Alternate function
    pc_afr[PC_INP] = 2                  # AF2 mode for TIM3 CH1 on PC6

    pa_moder[PA_GRN] = 1                # Output
    pa_odr[PA_GRN] = 1                  # LED on

    apb1enr[0] = 1                      # Timer 2 clock
    t2_psc[:] = psc_val - 1             # Set prescaler
    t2_arr[:] = tmr_sec - 1             # 1.0s timer period
    t2_ccer[TIM_CC1] = 0                # Disable CC1 for write to CCMR
    t2_ccmr[TIM_CC1] = 0x30             # CC1 compare mode, toggle o/p
    t2_ccer[TIM_CC1] = 1                # CC1 - enable OC1, disable OC1N
    t2_ccr1[:] = 0                      # Compare match value
    t2_cnt[:] = 0                       # Dummy load

    apb1enr[1] = 1                      # Timer 3 clock
    t3_psc[:] = psc_val - 1             # Set prescaler
    t3_ccer[TIM_CC1] = 0                # Disable CC1 for write to CCMR1
    t3_ccmr[TIM_CC1] = 0x41             # CC1 capture mode, filter 4
    t3_ccer[TIM_CC1] = 0b1011           # CC1 - enable IC1, both edges
    t3_sr[1] = 0                        # Clear capture flag

    t3_cr1[:] = 1                       # Enable input capture timer
    t2_cr1[:] = 1                       # Enable output compare timer

    last_value = None
    for _ in range (10):
        new_value = t3_capture()
        if last_value is not None:
            interval = (new_value - last_value) & 0xFFFF
            print("{:.2f} secs".format(interval / tmr_sec))
        last_value = new_value

    pa_odr[PA_GRN] = 0                  # LED off


# PWM output on external pin

def timer_pwm():
    ahb1enr[0] = 1                      # Port A clock

    pa_moder[PA_OUT] = 2                # Alternate function
    pa_afr[PA_OUT] = 1                  # AF1 mode for TIM2 CH1 on PA0

    pa_moder[PA_GRN] = 1                # Output
    pa_odr[PA_GRN] = 1                  # LED on

    apb1enr[0] = 1                      # Timer 2 clock
    t2_psc[:] = 10 - 1                  # Set fast prescaler
    t2_arr[:] = 100 - 1                 # Set PWM cycle period
    t2_cnt[:] = 0                       # Dummy load
    t2_ccer[TIM_CC1] = 0                # Disable CC1 for write to CCMR
    t2_ccmr[TIM_CC1] = 0x60             # CC1 PWM mode 1
    t2_ccer[TIM_CC1] = 1                # CC1 - enable OC1, disable OC1N
    t2_ccr1[:] = 50 - 1                 # Set initial PWM on time
    t2_cr1[:] = 1                       # Enable PWM timer

    while True:
        pct = input("Percentage on time? ")
        if not pct:
            break
        pct = int(pct)
        if 0 <= pct <= 100:
            t2_ccr1[:] = pct

    pa_odr[PA_GRN] = 0                  # LED off

