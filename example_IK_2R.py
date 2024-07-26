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


# Define the inverse kinematics function for 2R robot
def IK_2R(l1:float, l2:float, x:float, y:float, th1_est:float = 0, th2_est:float = 0, unit_ip:str = "rad", unit_op:str = "rad"):
    """
    @brief: Calculate the inverse kinematic solution of a 2R robot
    @param l1: length of the first link
    @param l2: length of the second link
    @param x: x coordinate of the end effector
    @param y: y coordinate of the end effector
    @param th1_est: estimated value of theta1, default = 0, default unit = rad
    @param th2_est: estimated value of theta2, default = 0, default unit = rad
    @param unit_op: unit of the output angles, "rad" or "deg", default = "rad"

    @return: [[theta1, theta2], [theta1_, theta2_]], theta2 for the first position has the same sign as th2_est

    """
    # Convert the input angles to radians if they are in degrees
    if unit_ip == "deg":
        th1_est = np.deg2rad(th1_est)
        th2_est = np.deg2rad(th2_est)


    # Distance of end effector from the origin
    d = np.sqrt(x**2 + y**2)

    # Incliniation angle of the end effector from the x-axis
    psi = np.arctan2(y, x)

    # Solving for cos theta2 gives us   
    cos_theta2 = (d**2 - l1**2 - l2**2) / (2 * l1 * l2)

    # Check if the end effector is reachable
    if (d > (l1 + l2)):
        print("End effector is farther than l1+l2, reaching the closest possible point")
        theta1 = psi
        theta2 = 0

        # Convert the output angles to degrees if required
        if unit_op == "deg":
            theta1 = np.rad2deg(theta1)
            theta2 = np.rad2deg(theta2)

        return [[theta1, theta2], [theta1, theta2]]
    
    elif (d < np.abs(l1 - l2)):
        print("End effector is closer than |l1-l2|, reaching the closest possible point")
        theta1 = psi
        theta2 = th2_est

        # Convert the output angles to degrees if required
        if unit_op == "deg":
            theta1 = np.rad2deg(theta1)
            theta2 = np.rad2deg(theta2)

        return [[theta1, theta2], [theta1, theta2]]
    else:
        # the end effector is reachable
        
        # The sign of theta2 is the same as th2_est (if provided)
        if th2_est < 0:
            theta2 = -np.arccos(cos_theta2)
            theta2_ = -theta2
        else:
            theta2 = np.arccos(cos_theta2)
            theta2_ = -theta2
        
        # Solving for theta1
        theta1 = psi - np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))
        theta1_ = psi - np.arctan2(l2 * np.sin(theta2_), l1 + l2 * np.cos(theta2_))

        # Convert the output angles to degrees if required
        if unit_op == "deg":
            theta1 = np.rad2deg(theta1)
            theta2 = np.rad2deg(theta2)
            theta1_ = np.rad2deg(theta1_)
            theta2_ = np.rad2deg(theta2_)

        return [[theta1, theta2], [theta1_, theta2_]]
    


BAUDRATE = 1000000
DEVICENAME = "/dev/ttyUSB0"  # "COM1" for Windows
ROBOT = "2R"
BASE_MOTOR = 1
ELBOW_MOTOR = 2

my2R = RobotAPI(DEVICENAME, BAUDRATE, ROBOT)
sleep(1)

# Robot parameters
l1 = 0.2
l2 = 0.2

# End effector position
x = 0.2
y = 0.2

# Estimated angles
th1e = 0
th2e = 0

# Inverse kinematics
sol = IK_2R(l1,l2,x,y,0,1,"rad","deg")

# Print the solution with three decimal places
print("Solution 1: ", np.round(sol[0], 3))
print("Solution 2: ", np.round(sol[1], 3))

# Go home
my2R.goHome()
sleep(1)

# Move the robot to the first solution
my2R.setJointAngle(BASE_MOTOR, sol[0][0], "deg")
my2R.setJointAngle(ELBOW_MOTOR, sol[0][1], "deg")
sleep(6)

# Move the robot to the second solution
my2R.setJointAngle(BASE_MOTOR, sol[1][0], "deg")
my2R.setJointAngle(ELBOW_MOTOR, sol[1][1], "deg")
sleep(8)

# Go home
my2R.goHome()
sleep(1)
