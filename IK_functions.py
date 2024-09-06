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