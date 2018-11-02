import RPi.GPIO as GPIO
import time as t

class servo:
    
    POS2DUTYCYCLE = 1/12.0    

    def __init__(self, pin):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        self.pin = GPIO.PWM(pin, 25) #50Hz
        self.pin.start(7.5) #%
        self.pos = 90 #deg
        self.__set_duty_cycle__(self.pos*self.POS2DUTYCYCLE)

    def destroy(self):
        self.pin.stop()
        GPIO.cleanup()

    def moveTo(self, pos):
        self.pos = pos
        self.__set_duty_cycle__(self.pos*self.POS2DUTYCYCLE)

    def moveDelta(self, delta):
        self.pos += delta
        self.__set_duty_cycle__(self.pos*self.POS2DUTYCYCLE)

    def update(self):
        self.__set_duty_cycle__(self.pos*self.POS2DUTYCYCLE)

    def __set_duty_cycle__(self, dc):
        if dc < 0:
            dc = 0
        if dc > 100:
            dc = 100
        self.pin.ChangeDutyCycle(dc)


if __name__ == '__main__':

    s = servo(18)    
    try:
        s.moveTo(0)
        while True:
            for i in range(170):
                s.moveTo(i)
                t.sleep(0.025)
            s.moveTo(90)
            t.sleep(1)
    finally:
        s.destroy()
