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
DEVICENAME = "/dev/ttyUSB0" # "COM1" for Windows
ROBOT = "2R"
BASE_MOTOR = 1
ELBOW_MOTOR = 2

my2R = RobotAPI(DEVICENAME, BAUDRATE, ROBOT)

my2R.setJointAngle(ELBOW_MOTOR, 10, "deg")
sleep(1)
my2R.penDown()
sleep(1)
my2R.goHome()

