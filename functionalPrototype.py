import RPi.GPIO as GPIO
import time
from RpiMotorLib import RpiMotorLib
from multiprocessing import Process
from adafruit_servokit import ServoKit

#####################################################################
# ADVANCED ENGINEERING DESIGN ME 4320
# IROBOT CONFIDENTIAL
# Team 11
# Members: Edward Jackson, Cole Kraus, James Mitchell, Max Reissner
#####################################################################


# Initial cleanup incase of error'd code
GPIO.cleanup()


## PIN INIT ##

# Stepper Motors
dir1 = 4
dir2 = 22
dir3 = 24
step1 = 27
step2 = 23
step3 = 25

# Encoder
encIn = 21

# Switch
btn = 26

## PIN INIT ##

## BOARD SETUP ##

GPIO.setmode(GPIO.BCM)
GPIO.setup(encIn, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(btn, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO_pins1 = (-1,-1,-1)
stepper1 = RpiMotorLib.A4988Nema(dir1, step1, GPIO_pins1, "A4988")
stepper2 = RpiMotorLib.A4988Nema(dir2, step2, GPIO_pins1, "A4988")
stepper3 = RpiMotorLib.A4988Nema(dir3, step3, GPIO_pins1, "A4988")

kit = ServoKit(channels=16)

## BOARD SETUP


######## HELPER FUNCTIONS ########

# Encoder callback function
# counts encoder reading as a value. Used to coordinate stepper motor and servo positioning
value = 0
def encoderValue(channel):
    global value
    value += 1
    value %= 100 # arbiturary value used to satisfy rate of motion 
    print(value)


GPIO.add_event_detect(encIn, GPIO.BOTH, callback=encoderValue, bouncetime=25)

    
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

# Helper functions for servo motors    
def servo1Func(angle):
    kit.servo[0].angle = angle
def servo2Func(angle):
    kit.servo[1].angle = angle
def servo3Func(angle):
    kit.servo[2].angle = angle  
    
# Multithreaded function used to move servos all together    
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

## Stepper Servo Pair Helper Functions ##    
def stepServo1(moved, dir1, rot, stepTime, grip):
    if moved is False:
        p1 = Process(target=stepper1Func, args=(dir1, rot, stepTime))
        p2 = Process(target=servo1Func, args=(grip,))
        p1.start()
        time.sleep(0.25) # abritrary value expirementally found
        p2.start()
        
def stepServo2(moved, dir2, rot, stepTime, grip):
    if moved is False:
        p1 = Process(target=stepper2Func, args=(dir2, rot, stepTime))
        p2 = Process(target=servo2Func, args=(grip,))
        p1.start()
        time.sleep(0.25)
        p2.start()

    
def stepServo3(moved, dir3, rot, stepTime, grip):
    if moved is False:
        p1 = Process(target=stepper3Func, args=(dir3, rot, stepTime))
        p2 = Process(target=servo3Func, args=(grip,))
        p1.start()
        time.sleep(0.25)
        p2.start()

## Stepper Servo Pair Helper Functions ##


######## HELPER FUNCTIONS ########

    
if __name__ == "__main__":
    
    # Stepper values
    dirDown = False
    dirUp = True
    rot = 45 # 45 steps
    stepTime = 0.0025 # expirementally found increment time for stepper steps

    # Servo values used for gripping. Expirementally found
    openGrip1 = 180
    openGrip2 = 160
    openGrip3 = 155
    closeGrip1 = 110
    closeGrip2 = 90
    closeGrip3 = 90
    
    print("Robot Starting! Waiting for Switch to be flipped!")
    try:
        
        GPIO.wait_for_edge(btn, GPIO.FALLING, bouncetime=100)
        print("Flip switched, engaging in hair spreading.")
        
        # Open Grippers to initialize.
        moveServos(openGrip1, openGrip2, openGrip3)
        
        # Initalize movements to false
        up1 = False
        up2 = False
        up3 = False
        down1 = False
        down2 = False
        down3 = False
        
        m = 15 # change motors every iteration
        s = 0.15 # sleep time
        
        

        # UNCOMMENT THIS CODE TO UTILIZE THE ENCODER
        #while True:
        #    #moveServos(closeGrip1, closeGrip2, closeGrip3)
        #    time.sleep(s)
            
        #    if value >= 0 and value < m:
                #print(up1)
        #        if up1 is False:
        #            stepServo1(up1, dirUp, rot, stepTime, openGrip1)
        #            up1 = True
        #            time.sleep(s)
        #        #time.sleep(5)
        #    elif value >= m and value < 2*m:
        #        if up2 is False:
        #           stepServo3(up2, dirUp, rot, stepTime, openGrip3)
        #            up2 = True
        #            time.sleep(s)
        #    elif value >= 2*m and value < 3*m:
        #        if up3 is False:
        #            stepServo2(up3, dirUp, rot, stepTime, openGrip2)
        #            up3 = True
        #            time.sleep(s)
        #    
        #    elif value >= 3*m and value < 4*m:
        #        if down1 is False:
        #            stepServo1(down1, dirDown, rot, stepTime, closeGrip1)
        #            down1 = True
        #            time.sleep(s)
        #    elif value >= 4*m and value < 5*m:
        #        if down2 is False:
        #            stepServo3(down2, dirDown, rot, stepTime, closeGrip3)
        #            down2 = True
        #            time.sleep(s)
        #    elif value >= 5*m and value < 6*m:
        #        if down3 is False:
        #            stepServo2(down3, dirDown, rot, stepTime, closeGrip2)
        #            down3 = True
        #            time.sleep(s)
        #    else:
        #        print("ran else")
        #        up1 = False
        #        up2 = False
        #        up3 = False
        #        down1 = False
        #        down2 = False
        #        down3 = False
        #        #pass
        # UNCOMMENT THIS CODE TO UTILIZE THE ENCODER
        
        
        # Hard coded solution 
        while True:
            moveServos(closeGrip1, closeGrip2, closeGrip3)
            time.sleep(s)
            
            stepServo1(up1, dirUp, rot, stepTime, openGrip1)
            time.sleep(s)
            stepServo3(up1, dirUp, rot, stepTime, openGrip3)
            time.sleep(s)
            stepServo2(up1, dirUp, rot, stepTime, openGrip2)
            time.sleep(s)
            
            stepServo1(up1, dirDown, rot+2, stepTime, openGrip1) # rot +2 to force the motors down
            time.sleep(s)
            stepServo3(up1, dirDown, rot+2, stepTime, openGrip3)
            time.sleep(s)
            stepServo2(up1, dirDown, rot+2, stepTime, openGrip2)
            time.sleep(s)
        
          
    except Exception as e:
        print("Fatal Exception\n" + str(e))
        print("Stopping the motors.")
        stepper1.motor_stop()
        stepper2.motor_stop()
        stepper3.motor_stop()
