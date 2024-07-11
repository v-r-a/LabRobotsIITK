# Important:
# USB Latency setting
# https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/#usb-latency-setting
# AX-12A motor
# https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
# For debugging, connect to Arduino Mega's HardWare Serial 1 UART port via USB-TTL converter

from RobotAPI_class import RobotAPI
import time
from time import sleep

BAUDRATE = 1000000
DEVICENAME = "/dev/ttyUSB0"  # "COM1" for Windows
ROBOT = "5bar"
LEFT_MOTOR = 1   # As seen from top and behind the robot
RIGHT_MOTOR = 2  # As seen from top and behind the robot

my5bar = RobotAPI(DEVICENAME, BAUDRATE, ROBOT)

my5bar.setJointAngle(RIGHT_MOTOR, 30, "deg")
sleep(1)
my5bar.penDown()
sleep(1)
my5bar.setTorqueOFF(1)
sleep(1)
my5bar.goHome()
      

