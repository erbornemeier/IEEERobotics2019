#!/usr/bin/python3
import smbus

class i2c_comm:

    def __init__(self, addr, bus=1, max_send_attempts=3):
        self.addr = addr
        self.bus = smbus.SMBus(bus)
        self.max_attempts = max_send_attempts

    def send_cmd(self, flag, data):
        for attempt in range(self.max_send_attempts):
            try:
                msg = self.serializeMsg(list(pos))
                self.bus.write_block_data(self.addr, flag, msg)
                return True
            except IOError:
                print("Attempt {}: can't communicate with arduino".format(attempt))
        return False

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
        i = -1
        while i != -1:
            i = int(input())
            comm.sendLEDPos((i, i+2))

    except KeyboardInterrupt:
        pass

