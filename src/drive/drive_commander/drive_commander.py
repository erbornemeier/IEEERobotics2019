#!/usr/bin/python3
from i2c_comm import i2c_comm

class drive_commander:
    def __init__(self, addr):
       self.i2c_comm = i2c_comm(addr)  

    def send_movement(self, dist, rot):
        return self.i2c_comm.send_cmd(0, [dist, rot])


if __name__ == "__main__":
    try:
        cmder = drive_commander(0x28)
        while True:
            dist = int(input("dist: "))
            rot  = int(input("rot: "))
            if not cmder.send_movement(dist, rot):
                break

    except KeyboardInterrupt:
        pass

