# Important:
# USB Latency setting
# https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/#usb-latency-setting
# AX-12A motor
# https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
# For debugging, connect to Arduino Mega's HardWare Serial 1 UART port via USB-TTL converter

from RobotAPI_class import RobotAPI
import time
from time import sleep

BAUDRATE = 1000000 # PC-Arduino communication rate
DEVICENAME = "COM9" # COM port number may vary
ROBOT = "2R" # Robot name (used in some functions)
BASE_MOTOR = 1 # Predefined ID of the base motor
ELBOW_MOTOR = 2 # Predefined ID of the elbow motor

# Make an instance of RobotAPI class
my2R = RobotAPI(DEVICENAME, BAUDRATE, ROBOT)

# Write your code here
