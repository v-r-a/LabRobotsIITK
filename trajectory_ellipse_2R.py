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


# Define the inverse kinematics function for 2R robot-----------
def IK_2R(l1: float, l2: float, x: float, y: float, th1_est: float = 0, th2_est: float = 0, unit_ip: str = "rad",
          unit_op: str = "rad"):
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
    d = np.sqrt(x ** 2 + y ** 2)

    # Incliniation angle of the end effector from the x-axis
    psi = np.arctan2(y, x)

    # Solving for cos theta2 gives us   
    cos_theta2 = (d ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)

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

# Cubic time trajectory-----------------------------------------
def cubic_time_traj(t:float, t1:float, t2:float, u1:float, u2:float, du1:float, du2:float):
    """
    @brief: Fits cubic u(t) = a0 + a1*t + a2*t^2 + a3*t^3
    @param t: current time
    @param t1: start time
    @param t2: end time
    @param u1: initial value
    @param u2: end value
    @param du1: time rate (derivative) of u at t1
    @param du2: time rate (derivative) of u at t2

    @return: u, a point on the cubic spline
    """
    # Warning if t is outside t1 and t2
    if t < t1:
        print("time t is less than initial time t1")
    if t > t2:
        print("time t is more than end time t2")

    # Calculation
    Amat = np.array([[1,t1,t1**2,t1**3],
                     [1,t2,t2**2,t2**3],
                     [0,1,2*t1,3*t1**2],
                     [0,1,2*t2,3*t2**2]])
    Bmat = np.array([u1,u2,du1,du2])
    sol = np.linalg.solve(Amat, Bmat) # Pending: error handling of singularity
    tvec = np.array([1,t,t**2,t**3])
    return np.dot(sol,tvec)

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
