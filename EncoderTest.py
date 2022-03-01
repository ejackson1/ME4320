import RPi.GPIO as GPIO
import time

GPIO.cleanup()

# Pin setup
in1 = 4
servoPIN = 27
GPIO.setmode(GPIO.BCM)

GPIO.setup(in1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(servoPIN, GPIO.OUT, initial=GPIO.LOW)

p = GPIO.PWM(servoPIN, 50) # set PWM with 50Hz

p.start(2.5)

def SetAngle(x, closeAngle, desiredServo):
    
    #duty = angle / 18 + 2.5
    #GPIO.output(servoPIN, True)
    #p.ChangeDutyCycle(duty)
    #sleep(1.5)
    #GPIO.output(servoPIN, False)
    #p.ChangeDutyCycle(0)
    
    if x <= closeAngle:
        angle = desiredServo/closeAngle * x
        duty = angle / 18 + 2.5
        p.ChangeDutyCycle(duty)
    elif x > closeAngle:
        angle = -desiredServo / (45 - closeAngle) * (x - closeAngle) + desiredServo
        duty = angle / 18 + 2.5
        print("angle " + str(angle))
        p.ChangeDutyCycle(duty)
    else:
        print(x)
        print("never should run")
        


try:
    x = 0
    while True:
        if GPIO.input(in1) == GPIO.HIGH:
            #print("high!")
            #time.sleep(0.05)
            
            if GPIO.input(in1) == GPIO.LOW:
                #print("low!")
                x += 1
                x %= 45
                print(x)
                #time.sleep(0.05)
                SetAngle(x, 27, 110)
                
                
                
                #if x % 45 == 0: # full rotation
                    #SetAngle(80)
            else:
                pass
                
        else:
            #print(x)
            pass
except:
    #p.stop()
    GPIO.cleanup()