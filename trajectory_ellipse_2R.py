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

# Cartesian trajectory functions-------------------------------------------

# Ellipse
def ellipse(cenXY: list, a: float, b: float, th: float):
    """
    @brief Gives out a point (X,Y) on ellipse as a function of parameter theta
    @param cenXY = [X,Y]
    @param a: length of the sem-major axis
    @param b: length of the semi-minor axis
    @param th: varying this parameter from 0 to 2pi traces an ellipse

    @return: [x,y], a point on ellipse corresponding to the parameter 'th'
    """
    return [cenXY[0] + a * np.cos(th), cenXY[1] + b * np.sin(th)]

# Straight line
def line_segment(x1y1: list, x2y2: list, u: float):
    """
    @brief: Gives out a point (X,Y) on the line segment a function of parameter 'u'
    @param x1y1: [x1,y1]
    @param x2y2: [x2,y2]
    @param u: varying this parameter from 0 to 1 traces the line segment

    @return: [x,y], a point on the line segment corresponding to the parameter 'u'
    """
    # extrapolation warning
    if u > 1:
        print("Warning: u > 1, extrapolated point.")
    if u < 0:
        print("Warning: u < 0, extrapolated point.")
    # calculation
    x1 = x1y1[0]
    y1 = x1y1[1]
    x2 = x2y2[0]
    y2 = x2y2[1]
    x = u * x2 + (1 - u) * x1
    y = u * y2 + (1 - u) * y1
    return [x, y]

#---------------------------------------------------------------

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

# Time to trace
TT = 6.0 # sec
# Open loop trajectory tracking

# initial time
t = 0
# time step for sending commands
dt = 0.025
# parameter
w = 0
# initial value
w1 = 0
# final value
w2 = 6 * np.pi # trace the ellipse thrice
# initial and final parametric velocity
dw1 = 0.5
dw2 = 0.5
# velocity multiplication factor for better performance
kv = 2

# Trajectory details
cen = [0.3,-0.05]
a = 0.06
b = 0.06

# Reach the starting point of the curve
xy = ellipse(cen, a, b, w)
sol = IK_2R(l1, l2, xy[0], xy[1], 0, 1)
my2R.setRobotState([[sol[0][0],1],[sol[0][1],1]])
sleep(2)
my2R.penDown()
sleep(0.5)

# Track the trajectory
while t < (TT-dt):
    # Increment time
    t += dt
    # Calculate the parameter value based on time
    w = cubic_time_traj(t, 0, TT, w1, w2, dw1, dw2)
    # Calculate the cartesian coordinates based on the parameter
    xy = ellipse(cen, a, b, w)
    sol_next = IK_2R(l1, l2, xy[0], xy[1], 0, 1, "rad", "rad")
    print(f"{sol_next[0][0]:.3f},{sol_next[0][1]:.3f}")
    # Compute the joint angle velocity numerically
    v1 = kv*(sol_next[0][0] - sol[0][0]) / dt
    v2 = kv*(sol_next[0][1] - sol[0][1]) / dt
    # Move command
    my2R.setRobotState([[sol_next[0][0],v1],[sol_next[0][1],v2]])
    # Back up solution for velocity calculation of the next step
    sol = sol_next
    # Wait
    sleep(dt)

sleep(2)

# Go home
print("Homing the robot...")
my2R.goHome()
sleep(1)
