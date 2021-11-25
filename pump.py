import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BOARD)

GPIO.setup(21,GPIO.OUT)

GPIO.output(21,False)
time.sleep(2)

while True:
    GPIO.output(21,True)
    time.sleep(3)
    GPIO.output(21,False)

