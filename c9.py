#!/usr/bin/env micropython
#
# Copyright (c) 2020, Ian Chapman (Chapmip Consultancy)
#
# This software is licensed under the MIT license (see LICENSE.TXT)
#
# Examples inspired by Section 9 (I2C) of Udemy ARM Cortex course:
# "Embedded Systems Bare-Metal Programming Ground Up[tm] (STM32)" --
# https://www.udemy.com/course/embedded-systems-bare-metal-programming/
#
# Draws on low-level I/O class module (iol.py) for "bare-metal" access
# to STM32 memory locations and device registers
#
# Created on a Pyboard (PYB) v1.1 (with STM32F405RGT6 microcontroller)
# but should be readily adaptable to other Micropython systems
#
# External hardware requirements: Bosch BMP280 digital temperature and
# pressure sensor powered by +3.3V supply (VDD/VDDIO and GND) with CSB
# connected via a 10k pull-up resistor to +3.3V to select I2C bus mode,
# SCL/SCK connected to X9, SDA/SDI connected to X10, and SDO left open
#
# See README.md for further information


from micropython import const
import pyb
import iol


# Get bus clock frequency in Hz

apb1_clk = pyb.freq()[2]                # Get pclk1 value from tuple


# Port bit definitions

PB_AVDD = const(5)                      # Power to MMA7660 accelerometer
                                        # (built into Pyboard v1.1)

PB_SCL = const(6)                       # I2C SCL line (X9)
PB_SDA = const(7)                       # I2C SDA line (X10)


# Register definitions

ahb1enr = iol.Reg("RCC.AHB1ENR")
apb1enr = iol.Reg("RCC.APB1ENR")

pb_moder = iol.RegArr("GPIOB.MODER", 16, 2)
pb_afr = iol.RegArr("GPIOB.AFR0,AFR1", 8, 4)
pb_otyper = iol.Reg("GPIOB.OTYPER")
pb_pupdr = iol.RegArr("GPIOB.PUPDR", 16, 2)
pb_odr = iol.Reg("GPIOB.ODR", 16)

i2c_cr1 = iol.Reg("I2C1.CR1")
i2c_cr2 = iol.Reg("I2C1.CR2")
i2c_sr1 = iol.Reg("I2C1.SR1")
i2c_sr2 = iol.Reg("I2C1.SR2")
i2c_dr = iol.Reg("I2C1.DR")
i2c_ccr = iol.Reg("I2C1.CCR")
i2c_trise = iol.Reg("I2C1.TRISE")


# I2C bus characteristics

I2C_STD_FREQ_HZ = const(100000)         # Bus speeds (max)
I2C_FAST_FREQ_HZ = const(400000)

I2C_STD_RISE_NS = const(1000)           # SCL rise times (max)
I2C_FAST_RISE_NS = const(300)


# I2C slave addresses

I2C_ADDR_ACCEL = const(0x4C)            # Note: 7 bits only!
I2C_ADDR_BMP280 = const(0x76)


# Accelerometer register addresses

ACCEL_XOUT = const(0x00)
ACCEL_YOUT = const(0x01)
ACCEL_ZOUT = const(0x02)
ACCEL_TILT = const(0x03)
ACCEL_SRST = const(0x04)
ACCEL_SPCNT = const(0x05)
ACCEL_INTSU = const(0x06)
ACCEL_MODE = const(0x07)
ACCEL_SR = const(0x08)
ACCEL_PDET = const(0x09)
ACCEL_PD = const(0x0A)


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


# Test inbuilt accelerometer using Pyboard (pyb) Micropython I2C calls

def i2c_test_pyb():
    ahb1enr[1] = 1                      # Port B clock

    pb_moder[PB_AVDD] = 1               # Output
    pb_odr[PB_AVDD] = 0                 # Power off initially

    i2c = pyb.I2C(1)
    i2c.init(pyb.I2C.MASTER, baudrate=100000)

    pb_odr.dump()
    i2c_cr1.dump()
    print()

    print("APB1 clock = ", apb1_clk)
    print("FREQ = {} decimal".format(i2c_cr2[5:0]))
    print("FS = {} binary".format(i2c_ccr[15]))
    print("DUTY = {} binary".format(i2c_ccr[14]))
    print("CCR = {} decimal".format(i2c_ccr[11:0]))
    print("TRISE = {} decimal".format(i2c_trise[5:0]))
    print()

    print("Power off ->", i2c.scan())

    pb_odr[PB_AVDD] = 1                 # Power on
    pyb.delay(1)                        # Allow time for power up

    print("Power on ->", i2c.scan())

    pb_odr[PB_AVDD] = 0                 # Power off

    i2c.deinit()


# Calculate initial parameters for CR2, CCR and TRISE registers

def i2c_parms(fast_bus=False, fast_duty=False):
    if fast_duty and not fast_bus:
        raise ValueError("fast_duty has no effect if fast_bus is False")

    freq_field, remainder = divmod(apb1_clk, 1000000)
    if remainder != 0:
        freq_field += 1                  # Round up
    if not 2 <= freq_field <= 50:
        raise ValueError("freq_field not in allowed range of 2-50")
    cr2_init = freq_field

    if fast_bus:
        i2c_bus_freq = I2C_FAST_FREQ_HZ
        i2c_rise_ns = I2C_FAST_RISE_NS
        ccr_init = (1 << 15)            # Set F/S bit
        if fast_duty:
            ccr_init |= (1 << 14)       # Set DUTY bit
            ccr_div = 25
        else:
            ccr_div = 3
    else:                               # Standard bus
        i2c_bus_freq = I2C_STD_FREQ_HZ
        i2c_rise_ns = I2C_STD_RISE_NS
        ccr_init = 0
        ccr_div = 2

    ccr_field, remainder = divmod(apb1_clk, ccr_div * i2c_bus_freq)
    if remainder != 0:
        ccr_field += 1                  # Round up
    if fast_bus and fast_duty:
        if not 1 <= ccr_field <= 4095:
            raise ValueError("ccr_field not in allowed range of 1-4095")
    else:
        if not 4 <= ccr_field <= 4095:
            raise ValueError("ccr_field not in allowed range of 1-4095")
    ccr_init |= ccr_field

    trise_init = int(apb1_clk * i2c_rise_ns / 1E9) + 1
    if not 1 <= trise_init <= 63:
            raise ValueError("trise_init not in allowed range of 1-63")

    print("cr2_init = {} decimal".format(cr2_init))
    print("ccr_init = 0x{:04X}".format(ccr_init))
    print("ccr_field = {} decimal".format(ccr_field))
    print("trise_init = {} decimal".format(trise_init))
    print()

    calc_i2c_bus_hz = round(apb1_clk / (ccr_div * ccr_field))
    calc_i2c_rise_ns = round(((trise_init - 1) * 1E9) / apb1_clk)

    print("Actual I2C bus frequency = {} Hz".format(calc_i2c_bus_hz))
    print("Actual SCL rise time = {} ns".format(calc_i2c_rise_ns))
    print()

    return cr2_init, ccr_init, trise_init


# Initialise I2C bus for Master mode operation

def i2c_init(fast_bus=False, fast_duty=False):
    ahb1enr[1] = 1                      # Port B clock
    apb1enr[21] = 1                     # I2C1 clock

    pb_moder[PB_SCL] = 2                # Alternate functions
    pb_moder[PB_SDA] = 2

    pb_afr[PB_SCL] = 4                  # AF4 mode for I2C1 functions
    pb_afr[PB_SDA] = 4

    pb_otyper[PB_SCL] = 1               # Open-drain outputs
    pb_otyper[PB_SDA] = 1

    pb_pupdr[PB_SCL] = 1                # Pull-up resistors
    pb_pupdr[PB_SDA] = 1

    i2c_cr1[:] = (1 << 15)              # Toggle SWRST bit for reset
    i2c_cr1[15] = 0

    cr2_init, ccr_init, trise_init = i2c_parms(fast_bus, fast_duty)

    i2c_cr2[:] = cr2_init               # Set up with I2C parms
    i2c_ccr[:] = ccr_init
    i2c_trise[:] = trise_init

    i2c_cr1[0] = 1                      # Set PE to enable I2C operation


# Wait for I2C bus free

def i2c_await_free():
    while i2c_sr2[1]:                   # Wait for BUSY = 0
        pass


# Set up I2C bus start condition and await completion

def i2c_start():
    i2c_cr1[8] = 1                      # Set START bit
    while not i2c_sr1[0]:               # Wait for SB = 1
        pass


# Write I2C address and await completion (return True if ACK received)

def i2c_write_addr(addr):
    i2c_dr[:] = addr & 0xFF             # Send slave address
    while not any(i2c_sr1.bits(1, 10)): # Wait until ADDR = 1 or AF = 1
        pass
    if i2c_sr1[10]:
        i2c_sr1[10] = 0                 # Clear AF bit
        return False
    dummy = i2c_sr2[:]                  # Dummy read to clear ADDR bit
    return True


# Write I2C data byte and await completion (return True if ACK received)

def i2c_write_byte(val):
    i2c_dr[:] = val & 0xFF              # Send data byte
    while not any(i2c_sr1.bits(7, 10)): # Wait until TXE = 1 or AF = 1
        pass
    if i2c_sr1[10]:
        i2c_sr1[10] = 0                 # Clear AF bit
        return False
    return True


# Read I2C data byte and await completion

def i2c_read_byte(last):
    i2c_cr1[10] = 0 if last else 1      # Clear ACK if last byte
    while not i2c_sr1[6]:               # Wait for RXNE = 1
        pass
    val = i2c_dr[:] & 0xFF              # Get data byte
    return val                          # Return value


# Set up I2C bus stop condition and await completion

def i2c_stop():
    i2c_cr1[9] = 1                      # Set STOP bit
    i2c_await_free()                    # Await bus clear


# Poll I2C slave (return True if present)

def i2c_poll(slave_addr):
    addr8 = (slave_addr << 1)           # Create 8 bit address
    i2c_await_free()                    # Await bus clear
    i2c_start()                         # Send start condition
    ack = i2c_write_addr(addr8 | 0)     # Send slave addr with R/W = 0
    i2c_stop()                          # Send stop condition
    return ack


# Write value to I2C slave register

def i2c_write_reg(slave_addr, reg_addr, value):
    if slave_addr is None or reg_addr is None or value is None:
        raise ValueError("slave_addr, reg_addr, value must hold data")
    if not 0 <= slave_addr <= 0x7F:
        raise ValueError("slave_addr must be a 7-bit value (0-127)")
    if not 0 <= reg_addr <= 0xFF:
        raise ValueError("reg_addr must be an 8-bit value (0-255)")
    if not 0 <= value <= 0xFF:
        raise ValueError("value must be an 8-bit value (0-255)")

    addr8 = (slave_addr << 1)           # Create 8 bit address
    i2c_await_free()                    # Await bus clear
    i2c_start()                         # Send start condition
    ack = i2c_write_addr(addr8 | 0)     # Send slave addr with R/W = 0
    if ack:
        ack = i2c_write_byte(reg_addr)  # Send register address
    if ack:
        ack = i2c_write_byte(value)     # Send data byte
    i2c_stop()                          # Send stop condition
    if not ack:
        raise RuntimeError("ACK not received during i2c_write_reg")


# Read value from I2C slave register

def i2c_read_reg(slave_addr, reg_addr):
    if slave_addr is None or reg_addr is None:
        raise ValueError("slave_addr and reg_addr must hold data")
    if not 0 <= slave_addr <= 0x7F:
        raise ValueError("slave_addr must be a 7-bit value (0-127)")
    if not 0 <= reg_addr <= 0xFF:
        raise ValueError("reg_addr must be an 8-bit value (0-255)")

    addr8 = (slave_addr << 1)           # Create 8 bit address
    i2c_await_free()                    # Await bus clear
    i2c_start()                         # Send start condition
    ack = i2c_write_addr(addr8 | 0)     # Send slave addr with R/W = 0
    if ack:
        ack = i2c_write_byte(reg_addr)  # Send register address
    if ack:
        i2c_start()                     # Send repeated start condition
        ack = i2c_write_addr(addr8 | 1) # Send slave addr with R/W = 1
    if ack:
        val = i2c_read_byte(last=True)  # Get one and only data byte
    i2c_stop()                          # Send stop condition
    if not ack:
        raise RuntimeError("ACK not received during i2c_read_reg")
    return val                          # Return data byte


# Selections for I2C bus modes

i2c_bus_modes = ( ("Standard", False, False),
                  ("Fast", True, False),
                  ("Fast Duty", True, True), )


# Allow user to select I2C bus mode

def i2c_select_bus_mode():
    print("I2C bus modes:")
    num_modes = len(i2c_bus_modes)
    while True:
        for index, mode in enumerate(i2c_bus_modes):
            print("{} {}".format(index + 1, mode[0]))
        entry = input("Select mode (1-{}): ".format(num_modes))
        print()
        if entry.isdigit():
            choice = int(entry)
            if 1 <= choice <= num_modes:
                break
    selection = i2c_bus_modes[choice - 1]
    return selection[1], selection[2]


# Test internal I2C accelerometer on Pyboard 1.1

def i2c_test_int():
    ahb1enr[1] = 1                      # Port B clock

    pb_moder[PB_AVDD] = 1               # Output
    pb_odr[PB_AVDD] = 0                 # Power off initially

    fast, duty = i2c_select_bus_mode()  # Get mode selection
    i2c_init(fast, duty)                # Initialise I2C bus

    pb_odr[PB_AVDD] = 1                 # Power on
    pyb.delay(1)                        # Allow time for power up

    mode = i2c_read_reg(I2C_ADDR_ACCEL, ACCEL_MODE)
    print("Before write, mode =", mode)
    i2c_write_reg(I2C_ADDR_ACCEL, ACCEL_MODE, 0x01)
    mode = i2c_read_reg(I2C_ADDR_ACCEL, ACCEL_MODE)
    print("After write, mode =", mode)
    print()

    pyb.delay(100)                      # Allow time to settle down

    for reg in range(11):
        byte = i2c_read_reg(I2C_ADDR_ACCEL, reg)
        print("addr {:02X}: byte {:02X}".format(reg, byte))

    print()
    mode = i2c_read_reg(I2C_ADDR_ACCEL, ACCEL_MODE)
    print("Before write, mode =", mode)
    i2c_write_reg(I2C_ADDR_ACCEL, ACCEL_MODE, 0x00)
    mode = i2c_read_reg(I2C_ADDR_ACCEL, ACCEL_MODE)
    print("After write, mode =", mode)

    pb_odr[PB_AVDD] = 0                 # Power off


# Scan for active I2C devices on bus

def i2c_scan():
    ahb1enr[1] = 1                      # Port B clock

    pb_moder[PB_AVDD] = 1               # Output
    pb_odr[PB_AVDD] = 0                 # Power off initially

    i2c_init()                          # Initialise I2C bus

    pb_odr[PB_AVDD] = 1                 # Power on
    pyb.delay(1)                        # Allow time for power up

    for addr in range(2, 128):
        response = "ACK" if i2c_poll(addr) else ""
        print("{:02X} {}".format(addr, response))

    pb_odr[PB_AVDD] = 0                 # Power off


# Test external Bosch BMP280 digital temperature and pressure sensor

def i2c_test_ext():
    ahb1enr[1] = 1                      # Port B clock

    fast, duty = i2c_select_bus_mode()  # Get mode selection
    i2c_init(fast, duty)                # Initialise I2C bus

    status = i2c_read_reg(I2C_ADDR_BMP280, BMP280_ID)
    print("BMP280 chip ID = 0x{:02X}".format(status))

    mode = i2c_read_reg(I2C_ADDR_BMP280, BMP280_CTRL_MEAS)
    print("Before write, mode = 0x{:02X}".format(mode))
    i2c_write_reg(I2C_ADDR_BMP280, BMP280_CTRL_MEAS, 0x27)
    mode = i2c_read_reg(I2C_ADDR_BMP280, BMP280_CTRL_MEAS)
    print("After write, mode = 0x{:02X}".format(mode))
    print()

    for reg in range(0xFC, 0xF2, -1):
        byte = i2c_read_reg(I2C_ADDR_BMP280, reg)
        print("addr {:02X}: byte {:02X}".format(reg, byte))

    while True:
        temp_lo = i2c_read_reg(I2C_ADDR_BMP280, BMP280_TEMP_LSB)
        temp_hi = i2c_read_reg(I2C_ADDR_BMP280, BMP280_TEMP_MSB)
        temp_raw = (temp_hi << 8) | temp_lo

        press_lo = i2c_read_reg(I2C_ADDR_BMP280, BMP280_PRESS_LSB)
        press_hi = i2c_read_reg(I2C_ADDR_BMP280, BMP280_PRESS_MSB)
        press_raw = (press_hi << 8) | press_lo

        print()
        print("Raw temperature = {} decimal".format(temp_raw))
        print("Raw pressure = {} decimal".format(press_raw))
        print()

        input("Press [ENTER] to continue: ")
