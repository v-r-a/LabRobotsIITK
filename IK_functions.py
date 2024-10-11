import numpy as np


# Define the inverse kinematics function for 2R robot
def IK_2R(l1:float, l2:float, x:float, y:float, th1_est:float = 0, th2_est:float = 0, unit_ip:str = "rad", unit_op:str = "rad"):
    """
    @brief Calculate the inverse kinematic solution of a 2R robot
    @param l1: length of the first link in m
    @param l2: length of the second link in m
    @param x: x coordinate of the end effector in m
    @param y: y coordinate of the end effector in m
    @param th1_est: estimated value of theta1, default = 0, default unit = rad
    @param th2_est: estimated value of theta2, default = 0, default unit = rad
    @param unit_ip: unit of the th1_est and th2_est angles, "rad" or "deg", default = "rad"
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
        print("Expected end effector position is outside workspace, i.e., farther than l1+l2.")
        print("Reaching the closest possible point on line joining the origin and the expected end effector position.")
        theta1 = psi
        theta2 = 0

        # Convert the output angles to degrees if required
        if unit_op == "deg":
            theta1 = np.rad2deg(theta1)
            theta2 = np.rad2deg(theta2)

        return [[theta1, theta2], [theta1, theta2]]
    
    elif (d < np.abs(l1 - l2)):
        print("Expected end effector position is outside workspace, i.e., nearer than |l1-l2|.")
        print("Reaching the closest possible point on line joining the origin and the expected end effector position.")
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
    

    # Cubic time trajectory-----------------------------------------

# Define the inverse kinematics function for 5-bar robot
def IK_5bar(robot_dim:list, x:float, y:float, unit_op:str = "rad"):
    """
    @brief Calculate the inverse kinematic solution of a 5-bar robot
    @param robot_dim: [l_b,l_d,r_b,r_d,d] l_b: left base link, left distal link, right base link, right distal link, distance between motors
    @param x: x coordinate of the end effector in m
    @param y: y coordinate of the end effector in m
    @param unit_ip: unit of the th1_est and th2_est angles, "rad" or "deg", default = "rad"
    @param unit_op: unit of the output angles, "rad" or "deg", default = "rad"

    @return: [theta1, theta2] for the motors to run. These valyes are offset by 90deg the general convention

    """
    # Check robot architecture parameters, i.e., link lengths.
    if(len(robot_dim) == 5):
        [l_b, l_d, r_b, r_d, d] = robot_dim
    else:
        print("Incorrect robot_dim input. Five lengths needed. Check the function documentation")
        return -1

    # Perform inverse kinematics on the left 2R
    x_l, y_l = np.array([x, y]) + np.array([0, -d / 2])
    # print(x_l,y_l)
    th1est=0   # doesn't matter
    th2est=-1 # left elbow out
    sol_left = IK_2R(l_b,l_d,x_l,y_l,th1est,th2est,'rad',unit_op)
    # print(sol_left)
    # Perform inverse kinematics on the right 2R
    x_r, y_r = np.array([x, y]) + np.array([0, d / 2])
    # print(x_r,y_r)
    th1est=0  # doesn't matter
    th2est=1  # elbow angle sign opposite of the left 2R
    sol_right = IK_2R(r_b,r_d,x_r,y_r,th1est,th2est,'rad',unit_op)
    # print(sol_right)
    # Choose the first branch of theta 1s
    sol_5bar = [sol_left[0][0],sol_right[0][0]]
    return  sol_5bar

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


# Cartesian trajectory functions---------------------------------


def lemniscate(cenXY: list, a: float, b: float, t: float):
    """
    @brief Gives out a point (X,Y) on a lemniscate as a function of parameter t
    @param cenXY: [X, Y], the center of the lemniscate
    @param a: horizontal scaling factor
    @param b: vertical scaling factor
    @param t: parameter that traces the lemniscate, typically varies between -pi/2 to pi/2 for one loop

    @return: [x, y], a point on the lemniscate corresponding to the parameter 't'
    """
    denom = np.sin(t)**2 + 1
    x = a * np.cos(t) / denom
    y = b * np.sin(t) * np.cos(t) / denom
    return [cenXY[0] + x, cenXY[1] + y]


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