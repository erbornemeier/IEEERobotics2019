#!/usr/bin/python3
import smbus
import struct
import time as t

class i2c_comm:
    BUS_NUM = 1

    SEND_LED_POS = 0

    def __init__(self, addr):
        self.addr = addr
        self.bus = smbus.SMBus(i2c_comm.BUS_NUM)

    def sendLEDPos(self, pos):
        try:
            msg = self.serializeMsg(list(pos))
            self.bus.write_block_data(self.addr, i2c_comm.SEND_LED_POS, msg)
        except IOError:
            print("can't communicate with arduino")

    def serializeMsg(self, msg):
	new_msg = []
       	for num in msg:
	     new_msg.append(num >> 8 & 0xff)
	     new_msg.append(num & 0xff)
	return new_msg


# Use this to debug this connection
if __name__ == "__main__":
    try:
        comm = i2c_comm(0x28)
        i = 1
        while i != -1:
            i = input()
            comm.sendLEDPos((i, i+2))

    except KeyboardInterrupt:
        pass

