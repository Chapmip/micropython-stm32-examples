#!/usr/bin/env micropython
#
# Copyright (c) 2020, Ian Chapman (Chapmip Consultancy)
#
# This software is licensed under the MIT license (see LICENSE.TXT)
#
# Examples inspired by Section 10 (SPI) of Udemy ARM Cortex course:
# "Embedded Systems Bare-Metal Programming Ground Up[tm] (STM32)" --
# https://www.udemy.com/course/embedded-systems-bare-metal-programming/
#
# Draws on low-level I/O class module (iol.py) for "bare-metal" access
# to STM32 memory locations and device registers
#
# Note: This example exposes the limitations of "bare-metal" routines
# running on top of Micropython, as the SPI transmit/receive functions
# are unable to keep up with the high speed of the SPI bus, even at its
# slowest setting (160 KHz on the Pyboard), as flagged by the receiver
# overrun errors.  However, a workaround has been achieved using inline
# assembler code for the time-critical part of these functions to yield
# an execution speed that is able to keep up with the SPI bus even at
# its maximum speed (21 MHz on the Pyboard).
#
# Created on a Pyboard (PYB) v1.1 (with STM32F405RGT6 microcontroller)
# but should be readily adaptable to other Micropython systems
#
# External hardware requirements: Bosch BMP280 digital temperature and
# pressure sensor powered by +3.3V supply (VDD/VDDIO and GND) with CSB
# connected to Y5 and *also* via a 10k pull-down resistor to ground to
# select SPI bus mode, SCL/SCK connected to Y6, SDO connected to Y7,
# and SDA/SDI connected to Y8.  Use short connection wires (e.g. 10cm),
# especially if running the SPI bus close to its maximum speed!
#
# See README.md for further information


from micropython import const
import pyb
import iol


# Get bus clock frequency in Hz

apb1_clk = pyb.freq()[2]                # Get pclk1 value from tuple


# Port bit definitions

PB_NSS = const(12)                      # SPI slave CSB line (Y5)
PB_SCK = const(13)                      # SPI slave SCK line (Y6)
PB_MISO = const(14)                     # SPI slave SDO line (Y7)
PB_MOSI = const(15)                     # SPI slave SDI line (Y8)


# Register definitions

ahb1enr = iol.Reg("RCC.AHB1ENR")
apb1enr = iol.Reg("RCC.APB1ENR")

pb_moder = iol.RegArr("GPIOB.MODER", 16, 2)
pb_ospeedr = iol.RegArr("GPIOB.OSPEEDR", 16, 2)
pb_afr = iol.RegArr("GPIOB.AFR0,AFR1", 8, 4)
pb_odr = iol.Reg("GPIOB.ODR", 16)

spi_cr1 = iol.Reg("SPI2.CR1")
spi_cr2 = iol.Reg("SPI2.CR2")
spi_sr = iol.Reg("SPI2.SR")
spi_dr = iol.Reg("SPI2.DR")


# BMP280 register addresses (omitting calibration registers)

BMP280_TEMP_XLSB = const(0xFC)
BMP280_TEMP_LSB = const(0xFB)
BMP280_TEMP_MSB = const(0xFA)
BMP280_PRESS_XLSB = const(0xF9)
BMP280_PRESS_LSB = const(0xF8)
BMP280_PRESS_MSB = const(0xF7)
BMP280_CONFIG = const(0xF5)
BMP280_CTRL_MEAS = const(0xF4)
BMP280_STATUS = const(0xF3)
BMP280_RESET = const(0xE0)
BMP280_ID = const(0xD0)


# Get available SPI baud rates indexed by CR1 BR field (3 bits: 0-7)

def spi_get_baud():
    baud_list = []
    for exp in range(8):
        baud_list.append(apb1_clk // (2 << exp))
    return tuple(baud_list)


# Allow user to select SPI baud rate

def spi_select_baud():
    baud_rates = spi_get_baud()
    num_rates = len(baud_rates)
    print("SPI baud rates:")
    for index, baud in enumerate(baud_rates):
        print("{} {}".format(index, baud))
    while True:
        entry = input("Select baud rate (0-{}): ".format(num_rates - 1))
        print()
        if entry.isdigit():
            choice = int(entry)
            if 0 <= choice <= num_rates - 1:
                break
    return choice


# Initialise SPI bus for Master mode operation

def spi_init(br_exp=7, spi_mode=0):
    if not 0 <= br_exp <= 7:
        raise ValueError("SPI baud rate selection must be in range 0-7")
    if not 0 <= spi_mode <= 3:
        raise ValueError("SPI mode must be in range 0-3")

    ahb1enr[1] = 1                      # Port B clock
    apb1enr[14] = 1                     # SPI2 clock

    pb_moder[PB_NSS] = 1                # Conventional output mode
    pb_ospeedr[PB_NSS] = 2              # High speed output
    pb_odr[PB_NSS] = 1                  # /SS is initially high

    pb_moder[PB_SCK] = 2                # Alternate functions
    pb_moder[PB_MOSI] = 2
    pb_moder[PB_MISO] = 2

    pb_ospeedr[PB_SCK] = 2              # High speed outputs
    pb_ospeedr[PB_MOSI] = 2

    pb_afr[PB_SCK] = 5                  # AF5 mode for SPI functions
    pb_afr[PB_MOSI] = 5
    pb_afr[PB_MISO] = 5

    spi_cr1[:] = spi_mode               # Set SPI mode, clear other bits
    spi_cr1[2] = 1                      # Set SPI master mode
    spi_cr1[5:3] = br_exp               # Set SPI baud rate
    spi_cr1[9:8] = 3                    # Set SPI SSM/SSI bits

    print("SPI2.CR1 set to 0x{:04X}".format(spi_cr1[:]))
    print()

    spi_cr2[:] = 0                      # Disable SPI interrupts
    spi_cr1[6] = 1                      # Enable SPI operation


# Send data byte to SPI bus: conceptual only (not useful in itself)

def spi_send_concept(data):
    if not 0 <= data <= 0xFF:
        raise ValueError("SPI write data {} is not a byte".format(data))

    while not spi_sr[1]:                # Wait for TXE = 1
        pass
    pb_odr[PB_NSS] = 0                  # Set /SS low to start sending
    spi_dr.write(data)
    while spi_sr[7]:                    # Wait for BUSY = 0
        pass
    pb_odr[PB_NSS] = 1                  # Return /SS high after sending


# Wait for SPI transmit buffer to be empty (TXE = 1)

def spi_await_txe():
    while not spi_sr[1]:                # Wait for TXE = 1
        pass


# Wait for SPI receive buffer to be filled (RXNE = 1) or overrun error

def spi_await_rxne():
    while True:
        if spi_sr[6]:                   # OVR = 1
            raise RuntimeError("Overrun error")
        if spi_sr[0]:                   # RXNE = 1
            break


# Wait for SPI bus not to be busy (BSY = 0)

def spi_await_not_busy():
    while spi_sr[7]:                    # Wait for BSY = 0
        pass


# Transmit tx_bytes to SPI bus and receive rx_bytes in return
# ** NOT FAST ENOUGH! (overrun error even at lowest baud rate) **

def spi_tx_rx_upy(tx_bytes):
    if not tx_bytes:
        raise ValueError("tx_bytes must contain at least one byte")
    if not all(0 <= elem <= 0xFF for elem in tx_bytes):
        raise ValueError("tx_bytes must contain 8-bit values (0-255)")

    rx_bytes = []                       # Start with blank return list
    spi_await_txe()
    pb_odr[PB_NSS] = 0                  # Set /SS low to start activity
    spi_dr.write(tx_bytes[0])           # Send first transmit byte

    for tx_byte in tx_bytes[1:]:
        spi_await_txe()
        spi_dr.write(tx_byte)           # Send next transmit byte
        spi_await_rxne()
        rx_byte = spi_dr.read()         # Get receive byte
        rx_bytes.append(rx_byte)

    spi_await_rxne()                    # ** Overrun error here **
    rx_byte = spi_dr.read()             # Get last receive byte
    rx_bytes.append(rx_byte)
    spi_await_txe()
    spi_await_not_busy()
    pb_odr[PB_NSS] = 1                  # Return /SS high after activity

    return rx_bytes


# Transmit two tx_bytes to SPI bus and receive two rx_bytes in return
# ** STILL NOT FAST ENOUGH! (overrun error even at lowest baud rate) **

def spi_tx_rx_upy_2(tx_byte_1, tx_byte_2):
    if tx_byte_1 is None or tx_byte_2 is None:
        raise ValueError("tx_bytes 1 and 2 must contain data")
    if not 0 <= tx_byte_1 <= 0xFF or not 0 <= tx_byte_2 <= 0xFF:
        raise ValueError("tx_bytes must contain 8-bit values (0-255)")

    spi_await_txe()
    pb_odr[PB_NSS] = 0                  # Set /SS low to start activity
    spi_dr.write(tx_byte_1)             # Send first transmit byte
    spi_await_txe()
    spi_dr.write(tx_byte_2)             # Send second transmit byte
    spi_await_rxne()
    rx_byte_1 = spi_dr.read()           # Get first receive byte
    spi_await_rxne()                    # ** Overrun error here **
    rx_byte_2 = spi_dr.read()           # Get second receive byte
    spi_await_txe()
    spi_await_not_busy()
    pb_odr[PB_NSS] = 1                  # Return /SS high after activity

    return rx_byte_1, rx_byte_2


# Inline assembly code version of spi_tx_rx_upy_2 method (for speed)
# Accepts: 16-bit union of 2 bytes (B1 << 8)|B2 to transmit to SPI bus
# Returns: 16-bit union of 2 bytes (B1 << 8)|B2 received from SPI bus
#          or 0xFFFFFFFF (-1) if receiver overrun error occurred

@micropython.asm_thumb
def spi_tx_rx_asm(r0):
    mov(r3, 0xFF)                       # Constant values
    movwt(r7, stm.GPIOB)
    movwt(r2, stm.SPI2)

    mov(r1, 8)                          # B3->x, B2->r6, B1->r5, B0->r4
    mov(r4, r0)
    and_(r4, r3)
    lsr(r0, r1)
    mov(r5, r0)
    and_(r5, r3)
    lsr(r0, r1)                         # B2 in r6: not used any further
    mov(r6, r0)
    and_(r6, r3)

    label(LOOP0)
    ldrh(r0, [r2, stm.SPI_SR])          # Wait for TXE = 1
    mov(r1, 1 << 1)
    tst(r0, r1)
    beq(LOOP0)

    movw(r1, 1 << PB_NSS)               # Assert /SS (low)
    strh(r1, [r7, stm.GPIO_BSRRH])

    mov(r0, r5)                         # Send B1 to SPI bus
    strh(r0, [r2, stm.SPI_DR])

    label(LOOP1)
    ldrh(r0, [r2, stm.SPI_SR])          # Wait for TXE = 1
    mov(r1, 1 << 1)
    tst(r0, r1)
    beq(LOOP1)

    mov(r0, r4)                         # Send B0 to SPI bus
    strh(r0, [r2, stm.SPI_DR])

    label(LOOP2)
    ldrh(r0, [r2, stm.SPI_SR])          # Wait for RXNE = 1 (or OVR err)
    mov(r1, 1 << 6)
    tst(r0, r1)
    bne(OVR_ERR)
    mov(r1, 1 << 0)
    tst(r0, r1)
    beq(LOOP2)

    ldrh(r0, [r2, stm.SPI_DR])          # Read data to r5 (B1)
    mov(r5, r0)
    and_(r5, r3)

    label(LOOP3)
    ldrh(r0, [r2, stm.SPI_SR])          # Wait for RXNE = 1 (or OVR err)
    mov(r1, 1 << 6)
    tst(r0, r1)
    bne(OVR_ERR)
    mov(r1, 1 << 0)
    tst(r0, r1)
    beq(LOOP3)

    ldrh(r0, [r2, stm.SPI_DR])          # Read data to r4 (B0)
    mov(r4, r0)
    and_(r4, r3)

    label(LOOP4)
    ldrh(r0, [r2, stm.SPI_SR])          # Wait for TXE = 1
    mov(r1, 1 << 1)
    tst(r0, r1)
    beq(LOOP4)

    label(LOOP5)
    ldrh(r0, [r2, stm.SPI_SR])          # Wait for BSY = 0
    mov(r1, 1 << 7)
    tst(r0, r1)
    bne(LOOP5)

    mov(r6, 0)                          # Clear r6 for return value B2

    mov(r1, 8)                          # 0->B3, r6->B2, r5->B1, r4->B0
    mov(r0, r6)
    lsl(r0, r1)
    orr(r0, r5)
    lsl(r0, r1)
    orr(r0, r4)

    b(DONE)                             # Skip round error code

    label(OVR_ERR)
    ldrh(r0, [r2, stm.SPI_DR])          # Clear OVR condition
    ldrh(r0, [r2, stm.SPI_SR])
    movwt(r0, 0xFFFFFFFF)               # Return error value (-1)

    label(DONE)
    movw(r1, 1 << PB_NSS)               # De-assert /SS (high)
    strh(r1, [r7, stm.GPIO_BSRRL])


# Write value to SPI slave register

def spi_write_reg(reg_addr, tx_byte):
    if reg_addr is None or tx_byte is None:
        raise ValueError("reg_addr and tx_byte must hold data")
    if not 0 <= reg_addr <= 0xFF:
        raise ValueError("reg_addr must be an 8-bit value (0-255)")
    if not 0 <= tx_byte <= 0xFF:
        raise ValueError("tx_byte must be an 8-bit value (0-255)")

    addr8 = reg_addr & ~(1 << 7)        # Create 8-bit address for write
    data16 = (addr8 << 8) | tx_byte     # Create 16-bit data for SPI bus

    result = spi_tx_rx_asm(data16)      # Send to SPI bus and get result
    if result < 0:
        raise RuntimeError("Receiver overrun occurred on SPI bus")


# Read value from SPI slave register

def spi_read_reg(reg_addr):
    if reg_addr is None:
        raise ValueError("reg_addr must hold data")
    if not 0 <= reg_addr <= 0xFF:
        raise ValueError("reg_addr must be an 8-bit value (0-255)")

    addr8 = reg_addr | (1 << 7)         # Create 8 bit address for read
    data16 = (addr8 << 8) | 0           # Create 16-bit data for SPI bus

    result = spi_tx_rx_asm(data16)      # Send to SPI bus and get result
    if result < 0:
        raise RuntimeError("Receiver overrun occurred on SPI bus")

    return result & 0xFF


# Test external Bosch BMP280 digital temperature and pressure sensor

def spi_test_ext():
    br_exp = spi_select_baud()
    spi_init(br_exp)

    status = spi_read_reg(BMP280_ID)
    print("BMP280 chip ID = 0x{:02X}".format(status))

    mode = spi_read_reg(BMP280_CTRL_MEAS)
    print("Before write, mode = 0x{:02X}".format(mode))
    spi_write_reg(BMP280_CTRL_MEAS, 0x27)
    mode = spi_read_reg(BMP280_CTRL_MEAS)
    print("After write, mode = 0x{:02X}".format(mode))
    print()

    pyb.delay(100)                      # Allow time to settle down

    for reg in range(0xFC, 0xF2, -1):
        byte = spi_read_reg(reg)
        print("addr {:02X}: byte {:02X}".format(reg, byte))

    while True:
        temp_lo = spi_read_reg(BMP280_TEMP_LSB)
        temp_hi = spi_read_reg(BMP280_TEMP_MSB)
        temp_raw = (temp_hi << 8) | temp_lo

        press_lo = spi_read_reg(BMP280_PRESS_LSB)
        press_hi = spi_read_reg(BMP280_PRESS_MSB)
        press_raw = (press_hi << 8) | press_lo

        print()
        print("Raw temperature = {} decimal".format(temp_raw))
        print("Raw pressure = {} decimal".format(press_raw))
        print()

        input("Press [ENTER] to continue: ")
