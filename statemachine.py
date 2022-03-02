from __future__ import division
import RPi.GPIO as GPIO
import time
import numpy as np
from RpiMotorLib import RpiMotorLib
from multiprocessing import Process
from threading import Thread, Lock
from adafruit_servokit import ServoKit



# Helper functions for stepper motors
def stepper1Func(direction, rot, stepTime):
    # direction [boolean] T: CCW   F: CW
    # rot [int] 200 is full rotation
    # stepTime [float] time taken to complete a step (0.0005 is fastest reliable time for full step)   
    stepper1.motor_go(direction, "Full", rot, stepTime, False, 0.0)
    return
def stepper2Func(direction, rot, stepTime): 
    stepper2.motor_go(direction, "Full", rot, stepTime, False, 0.0)
    return
def stepper3Func(direction, rot, stepTime): 
    stepper3.motor_go(direction, "Full", rot, stepTime, False, 0.0)
    return
    
def servo1Func(angle):
    kit.servo[0].angle = angle
    return
def servo2Func(angle):
    kit.servo[1].angle = angle
    return
def servo3Func(angle):
    kit.servo[2].angle = angle
    return
    
def allSteppersHigh(time_delay):
    GPIO.output(step1, GPIO.HIGH)
    GPIO.output(step2, GPIO.HIGH)
    GPIO.output(step3, GPIO.HIGH)
    time.sleep(time_delay)
    return
    
def allMotorsLow(time_delay):
    GPIO.output(step1, GPIO.LOW)
    GPIO.output(step2, GPIO.LOW)
    GPIO.output(step3, GPIO.LOW)
    time.sleep(time_delay)
    return
    
def stepper1F(direction, stepValue, delay_time):
    
    return
    
#mutex = Lock()    
def moveMotors(rot1, rot2, rot3,
               rot1D, rot2D, rot3D,
                 stepTime1, stepTime2, stepTime3,
                 grip1, grip2, grip3):
    if rot1D > 0:
        dir1 = False
        print("Dir1 is False")
    else:
        dir1 = True
        print("Dir1 is True")
    
    if rot2D > 0:
        dir2 = False
        print("Dir2 is False")
    else:
        dir2 = True
        print("Dir2 is True")
    
    if rot3D > 0:
        dir3 = False
        print("Dir3 is False")
    else:
        dir3 = True
        print("Dir3 is True")
 
    p1 = Process(target=stepper1Func, args=(dir1, rot1, stepTime1))
    p2 = Process(target=stepper2Func, args=(dir2, rot2, stepTime2))
    p3 = Process(target=stepper3Func, args=(dir3, rot3, stepTime3))
#        p4 = Process(target=servo1Func, args=(grip1,))
#        p5 = Process(target=servo2Func, args=(grip2,))
#        p6 = Process(target=servo3Func, args=(grip3,))
    p1.start()
    p2.start()
    p3.start()
#        p4.start()
#        p5.start()
#        p6.start()
    p1.join()
    p2.join()
    p3.join()
#        p4.join()
#        p5.join()
#        p6.join()
    #running = False
    return
    
def stepperEqn(enc):
    a = 22.5
    b = 4
    d = 67.5
    stepper1 = a*np.cos((enc) * np.pi / 180 ) + d
    stepper2 = a*np.cos(((enc) + 120) * np.pi/180) + d
    stepper3 = a*np.cos(((enc) + 240) * np.pi/180) + d
    
    step1D = -a*np.pi/180*np.sin(enc*np.pi/180)
    step2D = -(np.pi*a*np.sin(np.pi/180 * (enc + 120))) / 180
    step3D = -(np.pi*a*np.sin(np.pi/180 * (enc + 240))) / 180
    
    print("5/2 = " + str(5/2))
    
    return [int(stepper1), int(stepper2), int(stepper3), step1D, step2D, step3D]


def initialization():

    print("Press the Button when ready to begin spreading hair! \n Press Ctrl+C to close Program.")
    #GPIO.wait_for_edge(btn, GPIO.FALLING, bouncetime=100)
    print("Button Pressed. Initializing Motors. Standby.")
    time.sleep(2)
    
    # Test this asap so we don't break things!

    ## MOTOR INITS ##
    
    # Stepper Motors
    direction = False #Down
    stepTime = 0.005

    stepper1Func(direction, stepperEqn(0)[0], stepTime)
    stepper2Func(direction, stepperEqn(0)[1], stepTime)
    stepper3Func(direction, stepperEqn(0)[2], stepTime)

    # Servo Motors
    openGrip1 = 170
    openGrip2 = 160
    openGrip3 = 155

    closeGrip1 = 110
    closeGrip2 = 90
    closeGrip3 = 90
    kit.servo[0].angle = openGrip1
    kit.servo[1].angle = openGrip2
    kit.servo[2].angle = openGrip3

    ## MOTOR INITS ##

    print("Motors have been initialized! Ready to start!")

    return

def moveSteppers():
    
    return


value = 0
oldValue = 0
def encoderValue(channel):
    global value
    global oldValue
    value += 1
    print(value)
    

    
    #stepTime = 0.05
    #openGrip1 = 170
    #openGrip2 = 160
    #openGrip3 = 155
    
    #t = Thread(target = moveMotors, args = (False, False, True,
    #                  stepperEqn(value)[0], stepperEqn(value)[1], stepperEqn(value)[2],
    #                   stepTime, stepTime, stepTime,
    #                   openGrip1, openGrip2, openGrip3))
    #t.start()

    #moveMotors(False, False, True,
    #                  stepperEqn(value)[0], stepperEqn(value)[1], stepperEqn(value)[2],
    #                   stepTime, stepTime, stepTime,
    #                   openGrip1, openGrip2, openGrip3)
    #value = 0 
    #stepper1Func(False, stepperEqn(value)[0], stepTime)
    #stepper2Func(False, stepperEqn(value)[1], stepTime)
    #stepper3Func(True, stepperEqn(value)[2], stepTime)
    
    #stepper1F(False, stepperEqn(value)[0], delay_time)
    
    
    #print("here")
    stepTime = 0.05
    openGrip1 = 170
    openGrip2 = 160
    openGrip3 = 155
                        
    


def main():

    try:
        initialization()
        
        print("Starting wave!")
        GPIO.add_event_detect(encIn, GPIO.FALLING, callback=encoderValue, bouncetime=5)

        #global value
        #moveSteppers()
        stepTime = 0.005
        openGrip1 = 170
        openGrip2 = 160
        openGrip3 = 155
        print(stepperEqn(30))
        moveMotors(stepperEqn(30)[0], stepperEqn(30)[1], stepperEqn(30)[2],
                   stepperEqn(30)[3], stepperEqn(30)[4], stepperEqn(30)[5],
            stepTime, stepTime, stepTime,
            openGrip1, openGrip2, openGrip3)
        print("a")
       # moveMotors(False, False, True,
       #     stepperEqn(50)[0], stepperEqn(50)[1], stepperEqn(50)[2],
       #     stepTime, stepTime, stepTime,
       #     openGrip1, openGrip2, openGrip3)
        print("b")
        #while True:
               
            
        
        #time.sleep(0.05)        
        
        
        

    except Exception as e:
        print("Fatal Exception\n" + str(e))
        stepper1.motor_stop()
        stepper2.motor_stop()
        stepper3.motor_stop()
        #GPIO.cleanup()


if __name__ == "__main__":
    # Cleanup any previous error'd code
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

    # Button
    btn = 26

    # Board Setup 
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(encIn, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(btn, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    # Servo motor setup
    kit = ServoKit(channels=16)

    # Stepper motor setup
    microStep_pins = (-1,-1,-1) # ignores microstepping
    stepper1 = RpiMotorLib.A4988Nema(dir1, step1, microStep_pins, "A4988")
    stepper2 = RpiMotorLib.A4988Nema(dir2, step2, microStep_pins, "A4988")
    stepper3 = RpiMotorLib.A4988Nema(dir3, step3, microStep_pins, "A4988")

    ## PIN INIT ##

    # Run main
    main()
