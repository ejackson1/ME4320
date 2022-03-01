import RPi.GPIO as GPIO
import time

GPIO.cleanup()

dir1 = 4
step1 = 27

dir2 = 22
step2 = 23

dir3 = 24
step3 = 25


stepsPerRev = 50

GPIO.setmode(GPIO.BCM)
GPIO.setup(dir1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(step1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(dir2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(step2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(dir3, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(step3, GPIO.OUT, initial=GPIO.LOW)

#GPIO.output(dir1, GPIO.HIGH) # counter clockwise

time.sleep(1)

def allMotorsHigh():
    GPIO.output(step1, GPIO.HIGH)
    #time.sleep(0.001)
    #GPIO.output(step2, GPIO.HIGH)
    #time.sleep(0.001)
    #GPIO.output(step3, GPIO.HIGH)
    
def allMotorsLow():
    GPIO.output(step1, GPIO.LOW)
    #time.sleep(0.001)
    #GPIO.output(step2, GPIO.LOW)
    #time.sleep(0.001)
    #GPIO.output(step3, GPIO.LOW)
    
    

try:
    x = 0.005
    for _step in range(stepsPerRev):
        print(_step)
        GPIO.output(step1, GPIO.HIGH)
        GPIO.output(step2, GPIO.HIGH)
        GPIO.output(step3, GPIO.HIGH)
        time.sleep(x)
        
        GPIO.output(step1, GPIO.LOW)
        GPIO.output(step2, GPIO.LOW)
        GPIO.output(step3, GPIO.LOW)
        time.sleep(x)
    
    GPIO.output(dir1, GPIO.HIGH) # counter clockwise
    GPIO.output(dir2, GPIO.HIGH)
    GPIO.output(dir3, GPIO.HIGH)
    #time.sleep(1)
    
    for _step in range(stepsPerRev):
        #print(_step)
        print(_step)
        GPIO.output(step1, GPIO.HIGH)
        GPIO.output(step2, GPIO.HIGH)
        GPIO.output(step3, GPIO.HIGH)
        time.sleep(x)
        
        GPIO.output(step1, GPIO.LOW)
        GPIO.output(step2, GPIO.LOW)
        GPIO.output(step3, GPIO.LOW)
        time.sleep(x)

        
        
except:
    GPIO.cleanup()
