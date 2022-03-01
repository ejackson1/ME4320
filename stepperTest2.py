import RPi.GPIO as GPIO
import time
from RpiMotorLib import RpiMotorLib
from multiprocessing import Process, Pool
from adafruit_servokit import ServoKit


GPIO.cleanup()

dir1 = 4
step1 = 27

dir2 = 22
step2 = 23

dir3 = 24
step3 = 25

encIn = 21

GPIO.setmode(GPIO.BCM)
GPIO.setup(encIn, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

value = 0
def encoderValue(channel):
    global value
    value += 1
    print(value)


#GPIO.add_event_detect(encIn, GPIO.BOTH, callback=encoderValue, bouncetime=10)

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
                 stepTime1, stepTime2, stepTime3,
                 grip1, grip2, grip3):
    p1 = Process(target=stepper1Func, args=(dir1, rot1, stepTime1))
    p2 = Process(target=stepper2Func, args=(dir2, rot2, stepTime2))
    p3 = Process(target=stepper3Func, args=(dir3, rot3, stepTime3))
    p4 = Process(target=servo1Func, args=(grip1,))
    p5 = Process(target=servo2Func, args=(grip2,))
    p6 = Process(target=servo3Func, args=(grip3,))
    p1.start()
    p2.start()
    p3.start()
    p4.start()
    p5.start()
    p6.start()
    p1.join()
    p2.join()
    p3.join()
    p4.join()
    p5.join()
    p6.join()
    

dir1 = False
dir2 = False
dir3 = False
rot1 = 50
rot2 = 50
rot3 = 50
stepTime1 = 0.05
stepTime2 = 0.05
stepTime3 = 0.05

grip1 = openGrip1
grip2 = openGrip2
grip3 = openGrip3

moveMotors(dir1, dir2, dir3, rot1, rot2, rot3,
           stepTime1, stepTime2, stepTime3,
           grip1, grip2, grip3)
#stepTime1, stepTime2, stepTime3 = 0.005



#print(time.time())
while True:
    rot = input("Please input rotation\n")
    rot = int(rot)
    p1 = Process(target=stepper1Func, args=(direction, rot, stepTime))
    p2 = Process(target=stepper2Func, args=(direction, rot, stepTime))
    #p3 = Process(target=stepper3Func)
    p1.start()
    p2.start()
    #p3.start()
    p1.join()
    p2.join()
    #p3.join()

#kit.servo[1].angle = openGrip2
#time.sleep(1)
#stepper2Func(False, 90, 0.0025)
#time.sleep(1)
#kit.servo[1].angle = closeGrip2
#time.sleep(1)
#stepper2Func(True,45, 0.0025)
#time.sleep(1)
#kit.servo[1].angle = openGrip2

#stepper1Func(False, 50, 0.0025)

#GPIO.cleanup()