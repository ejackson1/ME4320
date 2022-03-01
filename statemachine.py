import RPi.GPIO as GPIO
import time
from RpiMotorLib import RpiMotorLib
from multiprocessing import Process
from adafruit_servokit import ServoKit

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


def initialization():

    print("Press the Button when ready to begin spreading hair! \n Press Ctrl+C to close Program.")
    GPIO.wait_for_edge(btn, GPIO.FALLING, bouncetime=100)
    print("Button Pressed. Initializing Motors. Standby.")

    
    # Test this asap so we don't break things!

    ## MOTOR INITS ##
    
    # Stepper Motors
    direction = False #Down
    stepTime = 0.05
    stepper1Func(direction, 100, stepTime)
    stepper2Func(direction, 100, stepTime)
    stepper3Func(direction, 100, stepTime)

    # Servo Motors
    kit.servo[0].angle = 90
    kit.servo[1].angle = 90
    kit.servo[2].angle = 90

    ## MOTOR INITS ##

    print("Motors have been initialized! Ready to start!")

    return

value = 0
def encoderValue():
    global value
    value += 1
    return value


def main():

    try:
        initialization()

        GPIO.setup(btn, GPIO.BOTH, callback=encoderValue, bouncetime=10)
        
        while True:
            enc = encoderValue()
            print("Encoder: " + str(enc))

            # Send Expirementally found positions
            # TODO

    except Exception as e:
        print("Fatal Exception\n " + str(e))
        stepper1.motor_stop()
        stepper2.motor_stop()
        stepper3.motor_stop()
        GPIO.cleanup()


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
