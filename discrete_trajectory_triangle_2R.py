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

# movement speed
spd = 1
# error tolerance
tol = 2

# Plot a triangle
xy1 = [0.25,0.15]
xy2 = [0.2, -0.2]
xy3 = [0.3, -0.05]

# Discretise the straight line xy1-xy2 into 8 points
traj_points = []
q_traj = []
npts = 8
for i in range(npts):
    u = i * 1 / (npts - 1)
    traj_points.append(line_segment(xy1, xy2, u))
    sol = IK_2R(l1,l2,traj_points[i][0],traj_points[i][1]) # Solve IK, give two solutions
    q_traj.append(sol[1]) # Choose the first solution
for i in range(npts):
    u = i * 1 / (npts - 1)
    traj_points.append(line_segment(xy2, xy3, u))
    sol = IK_2R(l1,l2,traj_points[i+npts][0],traj_points[i+npts][1]) # Solve IK, give two solutions
    q_traj.append(sol[1]) # Choose the first solution
for i in range(npts):
    u = i * 1 / (npts - 1)
    traj_points.append(line_segment(xy3, xy1, u))
    sol = IK_2R(l1,l2,traj_points[i+2*npts][0],traj_points[i+2*npts][1]) # Solve IK, give two solutions
    q_traj.append(sol[1]) # Choose the first solution


# Reach those points
op = my2R.setRobotState([[q_traj[0][0],spd],[q_traj[0][1],spd]])
sleep(2)
for i in range(len(traj_points)):
    op = my2R.setRobotState([[q_traj[i][0],spd],[q_traj[i][1],spd]])
    e1 = 100
    e2 = 100
    # Wait until the robot reaches the pose
    while (abs(e1) + abs(e2)) > tol:
        ja1 = my2R.getJointAngle(BASE_MOTOR)
        sleep(0.05)
        ja2 = my2R.getJointAngle(ELBOW_MOTOR)
        sleep(0.05)
        e1 = q_traj[i][0] - ja1
        e2 = q_traj[i][1] - ja2
        # print(f"joint angle errors: {e1:.3f}, {e2:.3f}")
        op = my2R.setRobotState([[q_traj[i][0],spd],[q_traj[i][1],spd]])
        sleep(0.1)
    my2R.penDown()
    sleep(0.2)
    my2R.penUp()
    sleep(0.2)

sleep(2)
# Go home
print("Homing the robot...")
my2R.goHome()
sleep(1)
