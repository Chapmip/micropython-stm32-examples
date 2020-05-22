#!/usr/bin/env micropython
#
# Copyright (c) 2020, Ian Chapman (Chapmip Consultancy)
#
# This software is licensed under the MIT license (see LICENSE.TXT)
#
# Examples inspired by Section 4 (USART) of Udemy ARM Cortex course:
# "Embedded Systems Bare-Metal Programming Ground Up[tm] (STM32)" --
# https://www.udemy.com/course/embedded-systems-bare-metal-programming/
#
# Draws on low-level I/O class module (iol.py) for "bare-metal" access
# to STM32 memory locations and device registers
#
# Created on a Pyboard (PYB) v1.1 (with STM32F405RGT6 microcontroller)
# but should be readily adaptable to other Micropython systems
#
# External hardware requirement: "Loop-back" connection from X1 (PA0)
# to X2 (PA1) via series resistor (e.g. 1k) or jumper wire
#
# See README.md for further information


from micropython import const
import utime
import pyb
import iol


# Get APB1 clock frequency in Hz

apb1_clk = pyb.freq()[2]                # Get pclk1 value from tuple


# Port bit definitions

PA_TXD = const(0)                       # External pin X1
PA_RXD = const(1)                       # External pin X2


# Register definitions

pa_enr = iol.Reg("RCC.AHB1ENR")
ua_enr = iol.Reg("RCC.APB1ENR")

pa_moder = iol.RegArr("GPIOA.MODER", 16, 2)
pa_afr = iol.RegArr("GPIOA.AFR0,AFR1", 8, 4)
pa_pupdr = iol.RegArr("GPIOA.PUPDR", 16, 2)
pa_odr = iol.Reg("GPIOA.ODR")
pa_idr = iol.Reg("GPIOA.IDR")

ua_brr = iol.Reg("UART4.BRR")
ua_cr1 = iol.Reg("UART4.CR1")
ua_sr = iol.Reg("UART4.SR")
ua_dr = iol.Reg("UART4.DR")


# Calculate BRR value from clock and baud rate (assuming OVER8 = 0)

def calc_brr_val(baud_rate):
    clk_div = apb1_clk / baud_rate
    brr_val = round(clk_div)
    actual_baud_rate = apb1_clk / brr_val
    print("baud_rate = ", baud_rate, "--> clk_div =", clk_div)
    print("brr_val = ", brr_val,
          "--> actual baud rate = {:.2f}".format(actual_baud_rate))
    if brr_val >= (1 << 16):
        raise ValueError("baud_rate is too low")
    return brr_val


# Echo test

def echo_test():
    pa_enr[0] = 1                   # Port A clock
    pa_moder[PA_TXD] = 1            # Output

    pa_pupdr[PA_RXD] = 2            # Pull-down resistor
    pa_moder[PA_RXD] = 0            # Input

    for _ in range(10):
        pa_odr[PA_TXD] ^= 1
        pyb.delay(500)
        print("Input is", pa_idr[PA_RXD])


# Send char once USART is ready

def send_char(char):
    while ua_sr[7] == 0:            # Wait for TXE high
        pass
    ua_dr[:] = ord(char)            # Send character to USART TX


# TX test

def tx_test():
    pa_enr[0] = 1                   # Port A clock
    ua_enr[19] = 1                  # UART4 clock

    pa_afr[PA_TXD] = 8              # AF8 mode for UART4_TXD on PA0
    pa_moder[PA_TXD] = 2            # Alternate function

    pa_pupdr[PA_RXD] = 2            # Pull-down resistor
    pa_moder[PA_RXD] = 0            # Input

    ua_brr[:] = calc_brr_val(4800)
    ua_cr1[3] = 1                   # Enable TX
    ua_cr1[13] = 1                  # Enable USART

    start_ms = utime.ticks_ms()     # Prepare to measure time
    num_chars = 5000

    for _ in range(num_chars):      # Loop for a while
        send_char('0')

    finish_ms = utime.ticks_ms()
    duration = utime.ticks_diff(finish_ms, start_ms) / 1000
    print("Duration: {:.2f} secs".format(duration))

    time_per_char = duration / num_chars
    print("Time per character = {:.2f} ms".format(time_per_char * 1000))
    time_per_bit = time_per_char / 10
    rough_baud_rate = 1.0 / time_per_bit
    print("Rough baud rate = {:.0f}".format(rough_baud_rate))

    pa_moder[PA_TXD] = 1            # Tidy up output
    pa_odr[PA_TXD] = 0


# Wait for receive char from USART

def recv_char():
    while ua_sr[5] == 0:            # Wait for RXNE high
        pass
    return chr(ua_dr[:])            # Return char from USART RX


# RX test

def rx_test():
    pa_enr[0] = 1                   # Port A clock
    ua_enr[19] = 1                  # UART4 clock

    pa_afr[PA_TXD] = 8              # AF8 mode for UART4_TXD on PA0
    pa_moder[PA_TXD] = 2            # Alternate function

    pa_pupdr[PA_RXD] = 2            # Pull-down resistor
    pa_afr[PA_RXD] = 8              # AF8 mode for UART4_RXD on PA1
    pa_moder[PA_RXD] = 2            # Alternate function

    ua_brr[:] = calc_brr_val(4800)
    ua_cr1[3] = 1                   # Enable TX
    ua_cr1[2] = 1                   # Enable RX
    ua_cr1[13] = 1                  # Enable USART

    num_loops = 24

    for _ in range(num_loops):      # Loop for a while
        for char in "ABCDEFGHIJKLMNOPQRSTUVWXYZ.0123456789":
            send_char(char)
            ch = recv_char()
            print(ch, end='')
        print()

    pa_moder[PA_TXD] = 1            # Tidy up output
    pa_odr[PA_TXD] = 0

