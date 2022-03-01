import RPi.GPIO as GPIO
import time
from adafruit_servokit import ServoKit

GPIO.cleanup()

kit = ServoKit(channels=16)

openGrip1 = 170
openGrip2 = 170
openGrip3 = 155
#kit.servo[0].angle = openGrip1

#kit.servo[1].angle = 90
kit.servo[2].angle = 155

time.sleep(1)
print("a")

closeGrip1 = 110
closeGrip2 = 90
closeGrip3 = 90
#kit.servo[0].angle = closeGrip1
#kit.servo[1].angle = 85
#kit.servo[2].angle = 85
#kit.servo[3].angle = 180
#kit.servo[4].angle = 180

GPIO.cleanup()