# Important:
# USB Latency setting
# https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/#usb-latency-setting
# AX-12A motor
# https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
# For debugging, connect to Arduino Mega's HardWare Serial 1 UART port via USB-TTL converter

from RobotAPI_class import RobotAPI
import numpy as np
import time
from time import sleep

# Import the inverse kinematics functions
from IK_functions import *


BAUDRATE = 1000000
DEVICENAME = "COM9"
ROBOT = "2R"
BASE_MOTOR = 1
ELBOW_MOTOR = 2

my2R = RobotAPI(DEVICENAME, BAUDRATE, ROBOT)
sleep(1)

# Go home
print("Homing the robot...")
my2R.goHome()
sleep(5)

# Robot parameters
l1 = 0.2
l2 = 0.2

# End effector position in metre
x = 0.2
y = 0.2

# Estimated angles
th1e = 0
th2e = 1
# tolerance in tracking joint angles (degrees)
tol = 4

# Inverse kinematics
sol = IK_2R(l1,l2,x,y,0,1,"rad","deg")

# Print the solution with three decimal places
print("Solution 1 (unit: deg): ", np.round(sol[0], 3))
print("Solution 2: (unit: deg)", np.round(sol[1], 3))

# Move the robot to the first solution
print("Moving towards the first solution...")
my2R.setJointAngle(BASE_MOTOR, sol[0][0], "deg")
my2R.setJointAngle(ELBOW_MOTOR, sol[0][1], "deg")

# Check feedback
e1 = 100
e2 = 100
# Wait until the robot reaches the pose
while (abs(e1) + abs(e2)) > tol:
    ja1 = my2R.getJointAngle(BASE_MOTOR,"deg")
    sleep(0.05)
    ja2 = my2R.getJointAngle(ELBOW_MOTOR,"deg")
    sleep(0.05)
    e1 = sol[0][0] - ja1
    e2 = sol[0][1] - ja2
    print(f"joint angle errors: {e1:.3f}, {e2:.3f}")
    my2R.setJointAngle(BASE_MOTOR, sol[0][0], "deg")
    my2R.setJointAngle(ELBOW_MOTOR, sol[0][1], "deg")
    sleep(0.2)

# Mark
print("Achieved joint angles (deg):",[ja1,ja2])
sleep(1)
my2R.penDown()
sleep(0.5)
my2R.penUp()
sleep(1)

# Move the robot to the second solution
print("Moving towards the second solution...")
my2R.setJointAngle(BASE_MOTOR, sol[1][0], "deg")
my2R.setJointAngle(ELBOW_MOTOR, sol[1][1], "deg")

# Check feedback
e1 = 100
e2 = 100
# Wait until the robot reaches the pose
while (abs(e1) + abs(e2)) > tol:
    ja1 = my2R.getJointAngle(BASE_MOTOR, "deg")
    sleep(0.05)
    ja2 = my2R.getJointAngle(ELBOW_MOTOR, "deg")
    sleep(0.05)
    e1 = sol[1][0] - ja1
    e2 = sol[1][1] - ja2
    print(f"joint angle errors: {e1:.3f}, {e2:.3f}")
    my2R.setJointAngle(BASE_MOTOR, sol[1][0], "deg")
    my2R.setJointAngle(ELBOW_MOTOR, sol[1][1], "deg")
    sleep(0.2)

# Mark
print("Achieved joint angles (deg):",[ja1,ja2])
sleep(1)
my2R.penDown()
sleep(0.5)
my2R.penUp()
sleep(1)

# Go home
print("Homing the robot...")
my2R.goHome()
sleep(1)
