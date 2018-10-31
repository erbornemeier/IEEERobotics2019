import RPi.GPIO as GPIO
import time as t

class servo:
    
    POS2DUTYCYCLE = 1/12.0    

    def __init__(self, pin):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        self.pin = GPIO.PWM(pin, 50) #50Hz
        self.pin.start(7.5) #%
        self.pos = 90 #deg

    def destroy(self):
        self.pin.stop()
        GPIO.cleanup()

    def moveTo(self, pos):
        self.pos = pos
        self.pin.ChangeDutyCycle(self.pos*self.POS2DUTYCYCLE)

    def moveDelta(self, delta):
        self.pos += delta
        self.pin.ChangeDutyCycle(self.pos*self.POS2DUTYCYCLE)


if __name__ == '__main__':

    try:
        s = servo(17)    
        while True:
            for i in range(170):
                s.moveTo(i)
                t.sleep(0.025)
    finally:
        s.destroy()
