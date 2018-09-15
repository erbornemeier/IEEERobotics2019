#!/usr/bin/python3

# For documentation on smbus, look at the smbus c library
import smbus

MOTOR_BUS = 1
MOTOR_ADDR = 0x28

bus = smbus.SMBus(MOTOR_BUS)

bus.write_block_data(MOTOR_ADDR, 0, msg)

msg = 'a'
while (msg != '0'):
    msg = input("What do?")

    if msg == "write":
        # Do a write
        bus.write_byte(MOTOR_ADDR, int(msg))
    elif msg == "read":
        # Do a read

