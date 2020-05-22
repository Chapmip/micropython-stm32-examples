#!/usr/bin/env micropython
#
# Copyright (c) 2020, Ian Chapman (Chapmip Consultancy)
#
# This software is licensed under the MIT license (see LICENSE.TXT)
#
# Examples inspired by Section 6 (Interrupts) of Udemy ARM Cortex course
# "Embedded Systems Bare-Metal Programming Ground Up[tm] (STM32)" --
# https://www.udemy.com/course/embedded-systems-bare-metal-programming/
#
# Note: It is not readily possible to create "bare-metal" interrupt
# handlers in Micropython, so these examples either use the Micropython
# "callback" mechanism when available (external interrupt and timer) or
# just report the way that interrupts are set up by Micropython (USART)
#
# Draws on low-level I/O class module (iol.py) for "bare-metal" access
# to STM32 memory locations and device registers
#
# Created on a Pyboard (PYB) v1.1 (with STM32F405RGT6 microcontroller)
# but should be readily adaptable to other Micropython systems
#
# External hardware requirement: LED via series resistor (e.g. 330 ohm)
# to ground from X1 (PA0)
#
# See README.md for further information


import pyb
import iol


# Register definitions used throughout module

ahb1enr = iol.Reg("RCC.AHB1ENR")
apb1enr = iol.Reg("RCC.APB1ENR")

pa_moder = iol.RegArr("GPIOA.MODER", 16, 2)
pa_odr = iol.Reg("GPIOA.ODR", 16)


# Port bit definition used throughout module

PA_OUT = const(0)                       # External pin X1


# Interrupt callback variables

source = None
event = None
ctr = 0


# Stored state of last dump

old_state = {}


# Get value by name -- print it if initial value or changed

def diag(var_str):
    new_val = eval(var_str)
    old_val = old_state.get(var_str)
    if old_val is None:
        old_state[var_str] = new_val
        print("{} = {:X}".format(var_str, new_val))
    elif new_val != old_val:
        old_state[var_str] = new_val
        print("{}: {:X} -> {:X}".format(var_str, old_val, new_val))


# Dump initial or changed states of relevant registers

def dump(fields):
    for item in fields:
        diag(item)
    print()


# EXTERNAL INTERRUPT

# Register definitions

apb2enr = iol.Reg("RCC.APB2ENR")

pc_moder = iol.RegArr("GPIOC.MODER", 16, 2)
pc_pupdr = iol.RegArr("GPIOC.PUPDR", 16, 2)
pc_ospeedr = iol.RegArr("GPIOC.OSPEEDR", 16, 2)
pc_otyper = iol.Reg("GPIOC.OTYPER", 16)
pc_odr = iol.Reg("GPIOC.ODR", 16)

exti_cr = iol.RegArr("SYSCFG.EXTICR0,EXTICR1,EXTICR2,EXTICR3", 4, 4)
exti_imr = iol.Reg("EXTI.IMR")
exti_emr = iol.Reg("EXTI.EMR")
exti_rtsr = iol.Reg("EXTI.RTSR")
exti_ftsr = iol.Reg("EXTI.FTSR")
exti_pr = iol.Reg("EXTI.PR")


# Port bit definition

PC_INT = const(6)                       # External pin Y1


# Interrupt vector

EXTI9_5 = const(0x0000009C)             # Address of vector
exti_vec = iol.Mem(EXTI9_5, 32)         # Content of vector


# Relevant register field values

regs_ext_int = ( "apb2enr[14]", "ahb1enr[2]",
                 "pc_moder[PC_INT]", "pc_pupdr[PC_INT]",
                 "pc_ospeedr[PC_INT]", "pc_otyper[PC_INT]",
                 "exti_cr[PC_INT]", "exti_imr[PC_INT]",
                 "exti_emr[PC_INT]", "exti_rtsr[PC_INT]",
                 "exti_ftsr[PC_INT]", "exti_pr[PC_INT]",
                 "exti_vec[:]" )


# Interrupt callback function

def callback_ext_int(line):
    global source, event, ctr
    source = "Ext"
    event = line
    ctr += 1


# Set up for external interrupt

def int_external():
    global source, event, ctr
    ahb1enr[0] = 1                      # Port A clock
    pa_moder[PA_OUT] = 1                # Output
    pa_odr[PA_OUT] = 1                  # Drives ExtInt high

    print("Before setup...")
    dump(regs_ext_int)

    ext_int = pyb.ExtInt(pyb.Pin("Y1"), pyb.ExtInt.IRQ_FALLING,
                         pyb.Pin.PULL_UP, callback_ext_int)

    print("After setup...")
    dump(regs_ext_int)

    while True:
        new_state = input("Ext Int pin state (0/1)? ")
        if not new_state:
            break
        if new_state in ("0", "1"):
            pa_odr[PA_OUT] = int(new_state)
            print("{}: Event {}, Count = {}".format(source, event, ctr))

    print()
    print("After usage...")
    dump(regs_ext_int)

    ext_int.disable()

    print()
    print("After disabling interrupt...")
    dump(regs_ext_int)

    pa_odr[PA_OUT] = 0                  # Drives ExtInt low


# TIMER INTERRUPTS

# Register definitions

t2_psc = iol.Reg("TIM2.PSC", 16)
t2_arr = iol.Reg("TIM2.ARR")            # Full 32 bits for TIM2
t2_cr1 = iol.Reg("TIM2.CR1", 16)
t2_dier = iol.Reg("TIM2.DIER", 16)


# Interrupt vector

T2_GLBI = const(0x000000B0)             # Address of vector
t2_vec = iol.Mem(T2_GLBI, 32)           # Content of vector


# Relevant register field values

regs_tmr_int = ( "apb1enr[0]", "t2_psc[:]", "t2_arr[:]",
                 "t2_cr1[:]", "t2_dier[:]", "t2_vec[:]" )


# Interrupt callback function

def callback_tmr_int(context):
    global source, event, ctr
    source = "Timer"
    event = context
    ctr += 1


# Set up for timer interrupt

def int_timer():
    ahb1enr[0] = 1                      # Port A clock
    pa_moder[PA_OUT] = 1                # Output
    pa_odr[PA_OUT] = 1                  # LED on

    print("Before setup...")
    dump(regs_tmr_int)

    tmr_int = pyb.Timer(2, freq=4, callback=callback_tmr_int)

    print("After setup...")
    dump(regs_tmr_int)

    while True:
        update = input("Update status (Y/N)? ")
        if not update or update.upper()[0] != "Y":
            break
        print("{}: Event {}, Count = {}".format(source, event, ctr))

    print()
    print("After usage...")
    dump(regs_tmr_int)

    pa_odr[PA_OUT] = 0                  # LED off


# USART INTERRUPTS

# Register definitions

pa_afr = iol.RegArr("GPIOA.AFR0,AFR1", 8, 4)
pa_pupdr = iol.RegArr("GPIOA.PUPDR", 16, 2)

u4_brr = iol.Reg("UART4.BRR")
u4_cr1 = iol.Reg("UART4.CR1")


# Control register bit definitions

UC_UE = const(13)
UC_RXNEIE = const(5)
UC_TE = const(3)
UC_RE = const(2)


# Port bit definitions

PA_TXD = const(0)                       # External pin X1
PA_RXD = const(1)                       # External pin X2


# Interrupt vector

U4_GLBI = const(0x00000110)             # Address of vector
u4_vec = iol.Mem(U4_GLBI, 32)           # Content of vector


# Relevant register field values

regs_uart_int = ( "ahb1enr[0]", "apb1enr[19]",
                  "pa_afr[0]", "pa_moder[0]", "pa_pupdr[0]",
                  "pa_afr[1]", "pa_moder[1]", "pa_pupdr[1]",
                  "u4_brr[:]", "u4_cr1[:]", "u4_cr1[UC_UE]",
                  "u4_cr1[UC_RXNEIE]", "u4_cr1[UC_TE]",
                  "u4_cr1[UC_RE]", "u4_vec[:]" )


# Set up for UART interrupt

def int_uart():
    pa_moder[PA_RXD] = 0                # Input
    pa_pupdr[PA_RXD] = 1                # Pull-up resistor

    print("Before setup...")
    dump(regs_uart_int)

    uart4 = pyb.UART(4)
    uart4.init(4800, 8, None, 1, timeout=100, timeout_char=100)

    print("After setup...")
    dump(regs_uart_int)

    while True:
        message = input("Type a string to send: ")
        if not message:
            break
        uart4.write(message.encode())
        line = uart4.read()
        if not line:
            print("-- TIMEOUT --")
        else:
            print(line.decode())

    print()
    print("After usage...")
    dump(regs_uart_int)
