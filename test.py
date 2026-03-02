import RPi.GPIO as GPIO
from time import sleep

channel=12

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(channel, GPIO.OUT, initial=GPIO.LOW)

p = GPIO.PWM(12, 500)  # channel=12 frequency=50Hz
p.start(0)

p.ChangeDutyCycle(4)


