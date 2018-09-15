import RPi.GPIO as GPIO
import time

try:
    DUMP_PIN = 8

    GPIO.setmode(GPIO.BOARD)

    GPIO.setup(DUMP_PIN, GPIO.OUT)
    GPIO.output(DUMP_PIN, True)

    input("press any key to dump")

    GPIO.output(DUMP_PIN, False)
    time.sleep(.1)
    GPIO.output(DUMP_PIN, True)

except KeyboardInterrupt:
    GPIO.cleanup()
finally:
    GPIO.cleanup()
