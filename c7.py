#!/usr/bin/env micropython
#
# Copyright (c) 2020, Ian Chapman (Chapmip Consultancy)
#
# This software is licensed under the MIT license (see LICENSE.TXT)
#
# Examples inspired by Section 7 (ADC) of Udemy ARM Cortex course:
# "Embedded Systems Bare-Metal Programming Ground Up[tm] (STM32)" --
# https://www.udemy.com/course/embedded-systems-bare-metal-programming/
#
# Draws on low-level I/O class module (iol.py) for "bare-metal" access
# to STM32 memory locations and device registers
#
# Created on a Pyboard (PYB) v1.1 (with STM32F405RGT6 microcontroller)
# but should be readily adaptable to other Micropython systems
#
# External hardware requirement: 50k potentiometer (10k or 100k also
# okay) from +3.3V supply to ground, with wiper connected to X7 (PA6)
# via series resistor (e.g. 330 ohm) or jumper wire
#
# See README.md for further information


from micropython import const
import pyb
import iol


# Get bus clock frequency in Hz

apb2_clk = pyb.freq()[3]                # Get pclk2 value from tuple


# Port bit definitions

PA_GRN = const(14)                      # Internal LED

PA_AIN = const(6)                       # External pin X7


# Other ADC settings

ADC_BITS = const(12)                    # Resolution in bits
ADC_CONVS = const(1)                    # Number of conversions
ADC_SAMP_CYCS = const(84)               # Number of ADC sampling cycles


# Calculate derived ADC values

if 0 <= PA_AIN <= 7:
    ADC_CHAN = PA_AIN
else:
    raise ValueError("Only PA0-PA7 can be used for ADC input on Port A")

if ADC_BITS in (12, 10, 8, 6):
    ADC_RES = (12 - ADC_BITS) // 2
    ADC_MAX = (1 << ADC_BITS) - 1
else:
    raise ValueError("ADC resolution can only be 12, 10, 8 or 6")

if not 1 <= ADC_CONVS <= 16:
    raise ValueError("ADC number of conversions must be 1-16")

ADC_SQRL = const(17)                    # Sequence length field in SQR

ADC_SAMP_VALS = (3, 15, 28, 56, 84, 112, 144, 480)  # Valid settings
ADC_SMP = ADC_SAMP_VALS.index(ADC_SAMP_CYCS)        # SMP field is 0-7


# Temperature sensor

TEMP_AIN = const(16)                    # For STM32F40x and STM32F41x


# Register definitions

ahb1enr = iol.Reg("RCC.AHB1ENR")
apb2enr = iol.Reg("RCC.APB2ENR")

pa_moder = iol.RegArr("GPIOA.MODER", 16, 2)
pa_odr = iol.Reg("GPIOA.ODR", 16)

adc1_cr1 = iol.Reg("ADC1.CR1")
adc1_cr2 = iol.Reg("ADC1.CR2")
adc1_sqr = iol.RegArr("ADC1.SQR3, SQR2, SQR1", 6, 5)
adc1_smpr = iol.RegArr("ADC1.SMPR2, SMPR1", 10, 3)

adc1_sr = iol.Reg("ADC1.SR", 8)         # Read only (else must be 32)
adc1_dr = iol.Reg("ADC1.DR", 16)        # Read only (else must be 32)

adc_ccr = iol.Reg("ADC.CCR")            # Common control register ADC123


# Basic ADC example

def adc_basic():
    ahb1enr[0] = 1                      # Port A clock

    pa_moder[PA_GRN] = 1                # Output
    pa_odr[PA_GRN] = 1                  # LED on

    pa_moder[PA_AIN] = 3                # Analog input

    apb2enr[8] = 1                      # ADC1 clock
    adc1_cr2[:] = 0                     # Disable ADC1 initially
    adc1_cr1[25:24] = ADC_RES           # Set ADC resolution
    adc1_sqr[0] = ADC_CHAN              # Set channel for single read
    adc1_sqr[ADC_SQRL] = ADC_CONVS - 1  # Set number of conversions
    adc1_cr2[0] = 1                     # Enable ADC1

    for _ in range(180):
        adc1_cr2[30] = 1                # Fire off software conversion
        while adc1_sr[1] == 0:          # Wait for EOC flag
            pass
        ana_val = adc1_dr[:]            # Get analog conversion value
        ana_pct = ana_val * 100 / ADC_MAX
        print(" {}%".format(round(ana_pct)))
        pyb.delay(1000)

    pa_odr[PA_GRN] = 0                  # LED off


# Convert temperature ADC reading to temperature in Celsius

def get_celsius(adc_temp_val):
    voltage = (adc_temp_val * 3.3) / ADC_MAX
    celsius = ((voltage - 0.76) / 0.0025) + 25
    return celsius


# ADC temperature reading from CPU

def adc_temp():
    ahb1enr[0] = 1                      # Port A clock

    pa_moder[PA_GRN] = 1                # Output
    pa_odr[PA_GRN] = 1                  # LED on

    apb2enr[8] = 1                      # ADC1 clock
    adc_ccr[23] = 1                     # Enable temp sensor
    adc_ccr[22] = 0                     # Disable V-BAT
    adc1_cr2[:] = 0                     # Disable ADC1 initially
    adc1_cr1[25:24] = ADC_RES           # Set ADC resolution
    adc1_sqr[0] = TEMP_AIN              # Special channel for temp read
    adc1_smpr[TEMP_AIN] = ADC_SMP       # Set ADC sample time
    adc1_sqr[ADC_SQRL] = ADC_CONVS - 1  # Set number of conversions
    adc1_cr2[0] = 1                     # Enable ADC1

    for _ in range(10):
        adc1_cr2[30] = 1                # Fire off software conversion
        while adc1_sr[1] == 0:          # Wait for EOC flag
            pass
        ana_val = adc1_dr[:]            # Get analog conversion value
        celsius = get_celsius(ana_val)  # Convert to temperature
        print("{:.1f}C".format(celsius))
        pyb.delay(1000)

    pa_odr[PA_GRN] = 0                  # LED off


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


# Register definitions

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
t2_ccr2 = iol.Reg("TIM2.CCR2")          # Full 32 bits for TIM2


# Timer capture/compare channels (indexes to CCMR/CCER arrays)

TIM_CC1 = const(0)
TIM_CC2 = const(1)
TIM_CC3 = const(2)
TIM_CC4 = const(3)


# Automatic ADC readings using Timer trigger

def adc_auto():
    ahb1enr[0] = 1                      # Port A clock

    pa_moder[PA_GRN] = 1                # Output
    pa_odr[PA_GRN] = 1                  # LED on

    apb1enr[0] = 1                      # Timer 2 clock
    t2_psc[:] = psc_val - 1             # Set prescaler
    t2_arr[:] = (tmr_sec * 3) - 1       # 3.0s timer period

    t2_ccer[TIM_CC2] = 0                # Disable CC2 for write to CCMR
    t2_ccmr[TIM_CC2] = 0x68             # CC2 PWM mode 1, preload enable
    t2_ccer[TIM_CC2] = 1                # CC2 - enable OC1, disable OC1N
    t2_ccr2[:] = 50 - 1                 # Preload value for PWM

    t2_cnt[:] = 0                       # Dummy load
    t2_cr1[:] = 1                       # Enable timer

    apb2enr[8] = 1                      # ADC1 clock
    adc_ccr[23] = 1                     # Enable temp sensor
    adc_ccr[22] = 0                     # Disable V-BAT
    adc1_cr2[:] = 0                     # Disable ADC1 initially
    adc1_cr1[25:24] = ADC_RES           # Set ADC resolution
    adc1_sqr[0] = TEMP_AIN              # Special channel for temp read
    adc1_smpr[TEMP_AIN] = ADC_SMP       # Set ADC sample time
    adc1_sqr[ADC_SQRL] = ADC_CONVS - 1  # Set number of conversions
    adc1_cr2[29:28] = 1                 # EXTEN = 1: rising edge trigger
    adc1_cr2[27:24] = 3                 # EXTSEL = 3: Timer 2 CC2 event
    adc1_cr2[0] = 1                     # Enable ADC1

    for _ in range(10):
        while adc1_sr[1] == 0:          # Wait for EOC flag
            pass
        ana_val = adc1_dr[:]            # Get analog conversion value
        celsius = get_celsius(ana_val)  # Convert to temperature
        print("{:.1f}C".format(celsius))
        pyb.delay(1000)

    pa_odr[PA_GRN] = 0                  # LED off
