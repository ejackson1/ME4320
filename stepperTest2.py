import RPi.GPIO as GPIO
import time
from RpiMotorLib import RpiMotorLib
from multiprocessing import Process
from adafruit_servokit import ServoKit


GPIO.cleanup()

dir1 = 4
step1 = 27

dir2 = 22
step2 = 23

dir3 = 24
step3 = 25

encIn = 21

btn = 26

GPIO.setmode(GPIO.BCM)
GPIO.setup(encIn, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(btn, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


value = 0
def encoderValue(channel):
    global value
    value += 1
    value %= 95
    print(value)


GPIO.add_event_detect(encIn, GPIO.BOTH, callback=encoderValue, bouncetime=25)

kit = ServoKit(channels=16)

openGrip1 = 170
openGrip2 = 160
openGrip3 = 155

closeGrip1 = 110
closeGrip2 = 90
closeGrip3 = 90


# Fake pins to just instanstiate the class
GPIO_pins1 = (-1,-1,-1)
#GPIO_pins2 = (103,104,105)
#GPIO_pins3 = (106,107,108)

stepper1 = RpiMotorLib.A4988Nema(dir1, step1, GPIO_pins1, "A4988")
stepper2 = RpiMotorLib.A4988Nema(dir2, step2, GPIO_pins1, "A4988")
stepper3 = RpiMotorLib.A4988Nema(dir3, step3, GPIO_pins1, "A4988")

# True is up, False is Down
def motor1f():
    motor1.motor_go(False, "Full", 40, 0.0025, False, 0.05)
def motor2f():
    motor2.motor_go(False, "Full", 40, 0.0025, False, 0.05)
def motor3f():
    motor3.motor_go(False, "Full", 40, 0.0025, False, 0.05)
    
# Helper functions for stepper motors
def stepper1Func(direction, rot, stepTime):
    # direction [boolean] T: CCW   F: CW
    # rot [int] 200 is full rotation
    # stepTime [float] time taken to complete a step (0.0005 is fastest reliable time for full step)
    stepper1.motor_go(direction, "Full", rot, stepTime, False, 0.0)
def stepper2Func(direction, rot, stepTime):
    stepper2.motor_go(direction, "Full", rot, stepTime, False, 0.0)
def stepper3Func(direction, rot, stepTime):
    stepper3.motor_go(direction, "Full", rot, stepTime, False, 0.0)
    
def servo1Func(angle):
    kit.servo[0].angle = angle
def servo2Func(angle):
    kit.servo[1].angle = angle
def servo3Func(angle):
    kit.servo[2].angle = angle


direction = False
stepTime = 0.005


def moveMotors(dir1, dir2, dir3,
                 rot1, rot2, rot3,
                 stepTime1, stepTime2, stepTime3):
    p1 = Process(target=stepper1Func, args=(dir1, rot1, stepTime1))
    p2 = Process(target=stepper2Func, args=(dir2, rot2, stepTime2))
    p3 = Process(target=stepper3Func, args=(dir3, rot3, stepTime3))
    
    if upDown is True:
        p1.start()
        time.sleep(0.2)
        p2.start()
        
        time.sleep(0.2)
        p3.start()

    else:
        p3.start()
        time.sleep(0.2)
        p2.start()
        time.sleep(0.2)
        p1.start()
    p1.join()
    p2.join()
    p3.join()
    
    
    
    
def moveServos(grip1, grip2, grip3):
    
    p4 = Process(target=servo1Func, args=(grip1,))
    p5 = Process(target=servo2Func, args=(grip2,))
    p6 = Process(target=servo3Func, args=(grip3,))
    p4.start()
    p5.start()
    p6.start()
    p4.join()
    p5.join()
    p6.join()
    
def stepServo1(moved, dir1, rot, stepTime, grip):
    if moved is False:
        print("ran true")
        p1 = Process(target=stepper1Func, args=(dir1, rot, stepTime))
        p2 = Process(target=servo1Func, args=(grip,))
        p1.start()
        time.sleep(0.3)
        p2.start()
        
        #return True
        
    #p1.join()
    #p2.join()

def stepServo2(moved, dir2, rot, stepTime, grip):
    if moved is False:
        p1 = Process(target=stepper2Func, args=(dir2, rot, stepTime))
        p2 = Process(target=servo2Func, args=(grip,))
        p1.start()
        time.sleep(0.3)
        p2.start()
    #p1.join()
    #p2.join()
    
def stepServo3(moved, dir3, rot, stepTime, grip):
    if moved is False:
        p1 = Process(target=stepper3Func, args=(dir3, rot, stepTime))
        p2 = Process(target=servo3Func, args=(grip,))
        p1.start()
        time.sleep(0.3)
        p2.start()
    #p1.join()
    #p2.join()
    
    
dirDown = False
dirUp = True
rot = 45
stepTime1 = 0.05
stepTime = 0.0025

openGrip1 = 180
openGrip2 = 160
openGrip3 = 155

closeGrip1 = 110
closeGrip2 = 90
closeGrip3 = 90

GPIO.wait_for_edge(btn, GPIO.FALLING, bouncetime=100)
print("wowo")

moveServos(openGrip1, openGrip2, openGrip3)
#time.sleep(1)


print("start")
try:
    
    #moveServos(closeGrip1, closeGrip2, closeGrip3)
    #time.sleep(0.1)
    #moveMotors(dirUp, dirUp, dirUp, rot, rot, rot,
    #           stepTime, stepTime, stepTime)
    #time.sleep(0.1)
    #moveServos(openGrip1, openGrip2, openGrip3)
    #time.sleep(0.5)
    #moveMotors(dirDown, dirDown, dirDown, rot, rot, rot,
    #           stepTime, stepTime, stepTime)
    up1 = False
    up2 = False
    up3 = False
    down1 = False
    down2 = False
    down3 = False
    
    m = 15
    s = 0.2
    
    
    while True:
        #moveServos(closeGrip1, closeGrip2, closeGrip3)
        time.sleep(s)
        
        if value >= 0 and value < m:
            #print(up1)
            if up1 is False:
                stepServo1(up1, dirUp, rot, stepTime, openGrip1)
                up1 = True
                time.sleep(s)
            #time.sleep(5)
        elif value >= m and value < 2*m:
            if up2 is False:
                stepServo3(up2, dirUp, rot, stepTime, openGrip3)
                up2 = True
                time.sleep(s)
        elif value >= 2*m and value < 3*m:
            if up3 is False:
                stepServo2(up3, dirUp, rot, stepTime, openGrip2)
                up3 = True
                time.sleep(s)
        
        elif value >= 3*m and value < 4*m:
            if down1 is False:
                stepServo1(down1, dirDown, rot, stepTime, closeGrip1)
                down1 = True
                time.sleep(s)
        elif value >= 4*m and value < 5*m:
            if down2 is False:
                stepServo3(down2, dirDown, rot, stepTime, closeGrip3)
                down2 = True
                time.sleep(s)
        elif value >= 5*m and value < 6*m:
            if down3 is False:
                stepServo2(down3, dirDown, rot, stepTime, closeGrip2)
                down3 = True
                time.sleep(s)
        else:
            print("ran else")
            up1 = False
            up2 = False
            up3 = False
            down1 = False
            down2 = False
            down3 = False
            #pass
            
        
    #moveServos(openGrip1, openGrip2, openGrip3)
    
    
    
    
    
except:
    pass
    #GPIO.cleanup()
#stepTime1, stepTime2, stepTime3 = 0.005



#print(time.time())

#kit.servo[1].angle = openGrip2
#time.sleep(1)
#stepper2Func(False, 90, 0.0025)
#time.sleep(1)
#kit.servo[1].anglue = closeGrip2
#time.sleep(1)
#stepper2Func(True,45, 0.0025)
#time.sleep(1)
#kit.servo[1].angle = openGrip2

#stepper1Func(False, 50, 0.0025)

#GPIO.cleanup()