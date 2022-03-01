import RPi.GPIO as GPIO
#import sys
#import signal

encIn = 21

GPIO.setmode(GPIO.BCM)
GPIO.setup(encIn, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

value = 0
def encoderValue(channel):
    global value
    value += 1
    print(value)


GPIO.add_event_detect(encIn, GPIO.BOTH, callback=encoderValue, bouncetime=10)

