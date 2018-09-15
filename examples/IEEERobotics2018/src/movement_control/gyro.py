#!/bin/python
import smbus
import math
import time

class Gyro:

    MIN_INC = 3.0

    def __init__(self, addr):
        self.bus = smbus.SMBus(1)
        self.addr = addr
        self.power_mgmt_1 = 0x6b
        self.bus.write_byte_data(self.addr, self.power_mgmt_1, 0)
        self.offset = 0
        self.calibrate()
        self.z_rot = 0.0

    def read_byte(self, adr):
        return self.bus.read_byte_data(self.addr, adr)

    def read_word(self, adr):
        high = self.bus.read_byte_data(self.addr, adr)
        low = self.bus.read_byte_data(self.addr, adr+1)
        val = (high << 8) + low
        return val

    def read_word_2c(self, adr):
        val = self.read_word(adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def dist(self,a,b):
        return math.sqrt((a*a)+(b*b))

    def get_x_rotation(self,x,y,z):
        radians = math.atan2(y, dist(x,z))
        return math.degrees(radians)

    def get_y_rotation(self,x,y,z):
        radians = math.atan2(x, dist(y,z))
        return -math.degrees(radians)

    def get_z_rotation(self):
        time.sleep(0.05)
        new_z = self.read_word_2c(0x47) + self.offset 
        self.z_rot += self.read_word_2c(0x47) + self.offset 
        return math.degrees(self.z_rot / (16384*4.32))

    def calibrate(self):
        z_error = 0
        for i in range(100):
                time.sleep(0.05)
                z_error += self.read_word_2c(0x47)
        self.offset = -(z_error / 100.0)

def main():
    g = Gyro(0x68)
    while True:
        print(g.get_z_rotation())

if __name__ == '__main__':
    main()
