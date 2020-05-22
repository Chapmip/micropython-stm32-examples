#!/usr/bin/env micropython
#
# Copyright (c) 2020, Ian Chapman (Chapmip Consultancy)
#
# This software is licensed under the MIT license (see LICENSE.TXT)
#
# Calculates STM32 clock frequencies from external crystal value and
# configuration settings, then compares them with system reported values
#
# Draws on low-level I/O class module (iol.py) for "bare-metal" access
# to STM32 memory locations and device registers
#
# Created on a Pyboard (PYB) v1.1 (with STM32F405RGT6 microcontroller)
# but should be readily adaptable to other Micropython systems
#
# See README.md for further information


from micropython import const
import pyb
import iol


# Put crystal frequency here

XTAL_FREQ = const(12000000)


# RCC clock configuration registers

pll_cfgr = iol.Reg("RCC.PLLCFGR")
rcc_cfgr = iol.Reg("RCC.CFGR")


# Obtain PLL field values

pll_q = pll_cfgr[27:24]
pll_p = pll_cfgr[17:16]
pll_n = pll_cfgr[14:6]
pll_m = pll_cfgr[5:0]


# Calculate PLL multiplier and divider factors

pll_p_div = (1 << (pll_p + 1))

if not 2 <= pll_q <= 15:
    raise ValueError("pll_q is {}, not in range 2-15}".format(pll_q))
pll_q_div = pll_q

if not 50 <= pll_n <= 432:
    raise ValueError("pll_n is {}, not in range 50-432}".format(pll_n))
pll_n_mult = pll_n

if not 2 <= pll_m <= 63:
    raise ValueError("pll_m is {}, not in range 2-63}".format(pll_m))
pll_m_div = pll_m


# Calculate VCO, PLL, USB and SYSCLK frequencies

vco_freq = XTAL_FREQ * (pll_n_mult / pll_m_div)
pll_freq = vco_freq / pll_p_div
usb_freq = vco_freq / pll_q_div
sysclk_freq = pll_freq


# Obtain bus clock field values

ppre2 = rcc_cfgr[15:13]
ppre1 = rcc_cfgr[12:10]
hpre = rcc_cfgr[7:4]


# Calculate bus clock divider factors

if hpre < 8:
    ahb_div = 1
else:
    ahb_div = (1 << (hpre - 7))                 # 0b1000 -> 2 etc.

if ppre1 < 4:
    apb1_div = 1
else:
    apb1_div = (1 << (ppre1 - 3))              # 0b100 -> 2 etc.

if ppre2 < 4:
    apb2_div = 1
else:
    apb2_div = (1 << (ppre2 - 3))              # 0b100 -> 2 etc.


# Calculate bus frequencies

ahb_freq = sysclk_freq / ahb_div
apb1_freq = sysclk_freq / apb1_div
apb2_freq = sysclk_freq / apb2_div


# Note that timer clocks are doubled if bus prescaler factor != 1
# as per section 7.2 ("Clocks") of RM0090 Reference Manual

if apb1_div == 1:
    tmr_apb1_clk = apb1_freq
else:
    tmr_apb1_clk = apb1_freq * 2

if apb2_div == 1:
    tmr_apb2_clk = apb2_freq
else:
    tmr_apb2_clk = apb2_freq * 2


# Compare with system reported values

calc_clocks = (int(sysclk_freq), int(ahb_freq),
               int(apb1_freq), int(apb2_freq))

sys_clocks = pyb.freq()

print("Calculated values: ", calc_clocks)
print("System reported values:", sys_clocks)
if calc_clocks == sys_clocks:
    print("Values match!")
else:
    print("Values DON'T match")

print()
print("APB1 timer clock =", int(tmr_apb1_clk))
print("APB2 timer clock =", int(tmr_apb2_clk))

