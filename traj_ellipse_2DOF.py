#--------------------------------------------------------------------------------------------
# Robot operation
ROBOT = "2bar" #or "2bar" ## Robot name (used in some functions)
BAUDRATE = 1000000 # PC-Arduino communication rate
DEVICENAME = "/dev/ttyUSB0" #"COM3" # COM port number may vary
MOTOR_ONE = 1 # Predefined ID of the LEFT motor
MOTOR_TWO = 2 # Predefined ID of the RIGHT motor

from RobotAPI_class import RobotAPI
import time,os,sys,subprocess
from time import sleep
import keyboard
import numpy as np
import matplotlib.pyplot as plt
# Import the inverse kinematics functions
from IK_functions import *
import pickle
import mujoco
import mujoco.viewer


# dir=os.path.dirname(os.path.abspath(__file__))
dir= os.getcwd() #cwd
#--------------------------------------------------------------------------------------------
# Offline trajectory planning (before sending it to the robot)

# Ellipse details
cen = [0.25, 0.1]
a = 0.04
b = 0.04

# Commands may be sent to the robot every 0.05 second (50 ms)
dt = 0.025

# Time to track one line segment
T = 6

# Time vector
t_vec = np.arange(0, T + dt, dt)

# Parameter vector: initial and final velocities to be zero
u_vec = np.array([cubic_time_traj(t, 0, T, 0, 4*np.pi, 0, 0) for t in t_vec])

# Plot u vs t in the first window
plt.figure()
plt.plot(t_vec, u_vec, label="u(t)", marker='o', linestyle=':')  # Circle markers
plt.xlabel("Time (s)")
plt.ylabel("u")
plt.title("Temporal planning: Cubic Time Trajectory: u vs t")
plt.grid(True)
plt.legend()
plt.show(block=False)  # Display the first plot without blocking

# Compute X(t) and Y(t) based on the line segment specifications
xy_vec = np.array([ellipse(cen,a,b,u) for u in u_vec])
x_vec = xy_vec[:, 0]
y_vec = xy_vec[:, 1]
# Calculate cartesian velocities using forward differences
x_dot_vec = np.diff(x_vec) / dt  # Angular velocity for th1_vec
y_dot_vec = np.diff(y_vec) / dt  # Angular velocity for th2_vec

# Plot the Cartesian trajectory in the second window
plt.figure()
plt.plot(x_vec, y_vec, label="Line Segment Path", marker='o', linestyle=':')
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Path planning: X-Y Trajectory (Line Segment)")
plt.grid(True)
plt.legend()
# plt.show(block=False)  # Keep both windows open indefinitely

if ROBOT == "5bar":
    # Robot architecture parameters
    arch_param_5bar = [0.2,0.2,0.2,0.2,0.067]

    # Perform inverse kinematics for 5-bar
    th1_th2_vec = np.array([IK_5bar(arch_param_5bar, x, y) for x, y in zip(x_vec, y_vec)])
    th1_vec = th1_th2_vec[:, 0]
    th2_vec = th1_th2_vec[:, 1]

    #MuJoCo digital twin
    # MuJoCo model and simulation data initialisation
    m = mujoco.MjModel.from_xml_path(os.path.dirname(__file__)+'/5bar.xml')
    d = mujoco.MjData(m)

elif ROBOT == "2bar":
    l1=0.2
    l2=0.2
    th1_th2_vec = np.array([IK_2R(l1, l2, x, y, 0, 1, "rad", "rad")[1] for x, y in zip(x_vec, y_vec)])
    th1_vec = th1_th2_vec[:, 0]
    th2_vec = th1_th2_vec[:, 1]

    #MuJoCo digital twin
    # MuJoCo model and simulation data initialisation
    m = mujoco.MjModel.from_xml_path(os.path.dirname(__file__)+'/2bar.xml')
    d = mujoco.MjData(m)

else:
    #open traj.pkl
    #load trajdata
    # trajdata=pickle.load(open(os.path.join(dir,'traj5bar.pkl'), 'rb'))
    trajdata=pickle.load(open(os.path.join(dir,'traj2bar.pkl'), 'rb'))
    x_vec,y_vec=trajdata[0],trajdata[1]
    t_vec, th1_vec,th2_vec=trajdata[2],trajdata[3],trajdata[4]
    th1_dot_vec,th2_dot_vec=trajdata[5],trajdata[6]

# Calculate angular velocities using forward differences
th1_dot_vec = np.diff(th1_vec) / dt  # Angular velocity for th1_vec
th2_dot_vec = np.diff(th2_vec) / dt  # Angular velocity for th2_vec

#save as .dat files
np.savetxt(os.path.join(dir,'th1des'+ROBOT+'.dat'), np.column_stack((t_vec, th1_vec)), delimiter='\t', header='Time\tth1_vec')
np.savetxt(os.path.join(dir,'th2des'+ROBOT+'.dat'), np.column_stack((t_vec, th2_vec)), delimiter='\t', header='Time\tth2_vec')
np.savetxt(os.path.join(dir,'xydes'+ROBOT+'.dat'), np.column_stack((x_vec, y_vec)), delimiter='\t', header='X\tY')

# Create a figure for multiple plots
plt.figure(figsize=(10, 8))

# Plot th1_vec vs u
plt.subplot(2, 2, 1)  # 2 rows, 2 columns, first subplot
plt.plot(u_vec, th1_vec, label='th1', marker='o', linestyle='-')
plt.xlabel('u')
plt.ylabel('th1_vec')
plt.title('th1_vec vs u')
plt.grid(True)
plt.legend()

# Plot th2_vec vs u
plt.subplot(2, 2, 2)  # 2 rows, 2 columns, second subplot
plt.plot(u_vec, th2_vec, label='th2', marker='o', linestyle='-')
plt.xlabel('u')
plt.ylabel('th2_vec')
plt.title('th2_vec vs u')
plt.grid(True)
plt.legend()

# Plot x vs u
plt.subplot(2, 2, 3)  # 2 rows, 2 columns, third subplot
plt.plot(u_vec, x_vec, label='x', marker='s', linestyle='-')
plt.xlabel('u')
plt.ylabel('x')
plt.title('x vs u')
plt.grid(True)
plt.legend()

# Plot y vs u
plt.subplot(2, 2, 4)  # 2 rows, 2 columns, fourth subplot
plt.plot(u_vec, y_vec, label='y', marker='s', linestyle='-')
plt.xlabel('u')
plt.ylabel('y')
plt.title('y vs u')
plt.grid(True)
plt.legend()

# Adjust layout and show the plots
plt.tight_layout()
plt.show(block=False)  # Keep the first set of plots open while we create the next set


# Create a figure for multiple plots
plt.figure(figsize=(10, 8))

# Plot th1_vec vs time with dual y-axes
ax1 = plt.subplot(2, 2, 1)  # 2 rows, 2 columns, first subplot
ax1.plot(t_vec, th1_vec, label='th1', color='b', marker='o', linestyle='-')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('th1_vec', color='b')
ax1.tick_params(axis='y', labelcolor='b')
ax1.grid(True)

# Create a second y-axis for th1_dot
ax1_twin = ax1.twinx()
ax1_twin.plot(t_vec[:-1], th1_dot_vec, label='th1_dot', color='r', marker='x', linestyle='--')
ax1_twin.set_ylabel('th1_dot', color='r')
ax1_twin.tick_params(axis='y', labelcolor='r')

# Add legends
ax1.legend(loc='upper left')
ax1_twin.legend(loc='upper right')

# Plot th2_vec vs time with dual y-axes
ax2 = plt.subplot(2, 2, 2)  # 2 rows, 2 columns, second subplot
ax2.plot(t_vec, th2_vec, label='th2', color='b', marker='o', linestyle='-')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('th2_vec', color='b')
ax2.tick_params(axis='y', labelcolor='b')
ax2.grid(True)

# Create a second y-axis for th2_dot
ax2_twin = ax2.twinx()
ax2_twin.plot(t_vec[:-1], th2_dot_vec, label='th2_dot', color='r', marker='x', linestyle='--')
ax2_twin.set_ylabel('th2_dot', color='r')
ax2_twin.tick_params(axis='y', labelcolor='r')

# Add legends
ax2.legend(loc='upper left')
ax2_twin.legend(loc='upper right')

# Plot x vs time with dual y-axes
ax3 = plt.subplot(2, 2, 3)  # 2 rows, 2 columns, third subplot
ax3.plot(t_vec, x_vec, label='x', color='b', marker='s', linestyle='-')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('x', color='b')
ax3.tick_params(axis='y', labelcolor='b')
ax3.grid(True)

# Create a second y-axis for x_dot
ax3_twin = ax3.twinx()
ax3_twin.plot(t_vec[:-1], x_dot_vec, label='x_dot', color='r', marker='x', linestyle='--')
ax3_twin.set_ylabel('x_dot', color='r')
ax3_twin.tick_params(axis='y', labelcolor='r')

# Add legends
ax3.legend(loc='upper left')
ax3_twin.legend(loc='upper right')

# Plot y vs time with dual y-axes
ax4 = plt.subplot(2, 2, 4)  # 2 rows, 2 columns, fourth subplot
ax4.plot(t_vec, y_vec, label='y', color='b', marker='s', linestyle='-')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('y', color='b')
ax4.tick_params(axis='y', labelcolor='b')
ax4.grid(True)

# Create a second y-axis for y_dot
ax4_twin = ax4.twinx()
ax4_twin.plot(t_vec[:-1], y_dot_vec, label='y_dot', color='r', marker='x', linestyle='--')
ax4_twin.set_ylabel('y_dot', color='r')
ax4_twin.tick_params(axis='y', labelcolor='r')

# Add legends
ax4.legend(loc='upper left')
ax4_twin.legend(loc='upper right')

# Adjust layout and show the plots
plt.tight_layout()
plt.show(block=False)
# plt.show()

# Keyboard callback
pendown = False
def keyboard_func(keycode):
  if chr(keycode) == ' ':
    global pendown
    pendown = not pendown


def mj_sim(m,d,ttraj,th1traj,th2traj,xy_vec,ti=2,isDone=True):
    t_fb = np.array(ttraj)
    th1_fb = np.array(th1traj)
    th2_fb = np.array(th2traj)
    t_mj = []
    th1mj=[]
    th2mj=[]
    xvec=[]
    yvec=[]
    # ti=2 #time to reach initial pose
    # iterator for decorative geometry objects
    idx_geom = 0

    with mujoco.viewer.launch_passive(m, d,key_callback=keyboard_func) as viewer:
        while viewer.is_running() and d.time <= t_fb[-1]+ti:
            step_start = time.time()
            # d.qpos[0] = th1_fb[0]
            # d.qpos[3] = th2_fb[1]
            # #Solve for qpos[1],qpos[2] 

            # mj_step can be replaced with code that also evaluates
            # a policy and applies a control signal before stepping the physics.
            if d.time <= ti:
                th1 = np.interp(0, t_fb, th1_fb)
                th2 = np.interp(0, t_fb, th2_fb)
            else:
                th1 = np.interp(d.time-ti, t_fb, th1_fb)
                th2 = np.interp(d.time-ti, t_fb, th2_fb)
            d.ctrl[0] = th1 #0.5*(th1-d.qpos[0]) - 0.008*d.qvel[0] #PD control
            d.ctrl[1] = th2 #0.5*(th2-d.qpos[3]) - 0.008*d.qvel[3] #PD control
            print(d.time)
            mujoco.mj_step(m, d)
            # Copy the pen site x-y coordinates
            end_effector_xy = d.site_xpos[0][0:2]
            if d.time > ti:
                t_mj.append(d.time-ti)
                xvec.append(end_effector_xy[0])
                yvec.append(end_effector_xy[1])
                th1mj.append(d.qpos[0])
                th2mj.append(d.qpos[m.actuator_trnid[1][0]])
                mujoco.mjv_initGeom(viewer.user_scn.geoms[idx_geom],
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=[0.003,0,0],
                pos = np.array([end_effector_xy[0], end_effector_xy[1], 0.0]),
                mat=np.eye(3).flatten(),
                rgba=np.array([1,0,0,0.5]))
                idx_geom += 1
                viewer.user_scn.ngeom = idx_geom
                # Reset if the number of geometries hit the limit
                if idx_geom > (viewer.user_scn.maxgeom - 50):
                    # Reset
                    idx_geom = 1
            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()

            """if pendown:
            my2DOFRobot.penDown(90)
            else:
            my2DOFRobot.penUp()
            """
            # Rudimentary time keeping, will drift relative to wall clock.
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

            # Operate the robot at ~60 Hz
            #my2DOFRobot.setJointAngle(LEFT_MOTOR, d.ctrl[0])
            #my2DOFRobot.setJointAngle(RIGHT_MOTOR, d.ctrl[1])
    if isDone:
        plt.figure()
        plt.plot(t_fb,th1_fb,'r.',label=r'\theta_{des_1}')
        plt.plot(t_mj,th1mj,'b-',label=r'\theta_{mj_1}')
        plt.plot(t_fb,th2_fb,'r.',label=r'\theta_{des_2}')                
        plt.plot(t_mj,th2mj,'b-',label=r'\theta_{mj_2}')
        plt.xlabel('Time (s)')
        plt.ylabel('Joint Angles (rad)')
        plt.grid()
        plt.legend()
        plt.title('Joint Angles vs Time in MuJoCo Simulation')
        #save xvec,yvec to dat file
        np.savetxt('xymj'+ROBOT+'.dat', np.column_stack((xvec, yvec)), delimiter='\t', header='X\tY')
        plt.figure()
        plt.plot(xy_vec[:,0],xy_vec[:,1],'g-',label='Desired Path')
        plt.plot(xvec,yvec,'r-',label='End-effector Path')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Robot End-effector Path in MuJoCo Simulation')
        plt.axis('equal')
        plt.grid()
        plt.legend()
        plt.show()
    else:
        d.time=0
        #reset model and data
        # mujoco.mj_resetData(m, d)
        return

def line_track(x1y1,x2y2,T):
    # T=1
    # x1y1 = [0.25, -0.1]
    # x2y2 = [0.25, 0.1]
    # Time vector    
    t_vec = np.arange(0, T + dt, dt)

    # Parameter vector: initial and final velocities to be zero
    u_vec = np.array([cubic_time_traj(t, 0, T, 0, 1, 0, 0) for t in t_vec])

    # Compute X(t) and Y(t) based on the line segment specifications
    xy_vec = np.array([line_segment(x1y1, x2y2, u) for u in u_vec])
    
    x_vec = xy_vec[:, 0]
    y_vec = xy_vec[:, 1]

    #cartesian to joint space
    if ROBOT == "5bar":
        # Robot architecture parameters
        arch_param_5bar = [0.2,0.2,0.2,0.2,0.067]

        # Perform inverse kinematics for 5-bar
        th1_th2_vec = np.array([IK_5bar(arch_param_5bar, x, y) for x, y in zip(x_vec, y_vec)])
        th1_vec = th1_th2_vec[:, 0]
        th2_vec = th1_th2_vec[:, 1]

    elif ROBOT == "2bar":
        l1=0.2
        l2=0.2
        th1_th2_vec = np.array([IK_2R(l1, l2, x, y, 0, 1, "rad", "rad")[1] for x, y in zip(x_vec, y_vec)])
        th1_vec = th1_th2_vec[:, 0]
        th2_vec = th1_th2_vec[:, 1]

    return th1_vec, th2_vec, t_vec, xy_vec


#Robot tracking 
# Data to send to the motors: Next position, Avg speed to next position
# th1_vec = th1_vec[1:]  # Remove the first element because the robot is already there
# th2_vec = th2_vec[1:]  # Remove the first element because the robot is already there
# t_vec = t_vec[1:]  # Adjust time vector accordingly

# Store feedback angles
th1_fb = np.zeros(len(th1_vec))
th2_fb = np.zeros(len(th1_vec))
th1_dot_fb = np.zeros(len(th1_vec))
th2_dot_fb = np.zeros(len(th1_vec))
t_fb = np.zeros(len(th1_vec))

try:
    # Make an instance of RobotAPI class
    my2DOFRobot = RobotAPI(DEVICENAME, BAUDRATE, ROBOT)

    print("The joint angles and joint velocities are within limits, press any key to proceed,\n Else, interrupt the program operation.")
    # key = keyboard.read_key()

    # Go to the start point and wait
    print("Moving to the first position...")
    spd = 1
    op = False
    while op == False:
        op = my2DOFRobot.setRobotState([[th1_vec[0],spd],[th1_vec[0],spd]])
        if op == False:
            print("Error in sending data, trying again...")

    # Check feedback
    tol = 0.05
    e1 = 100
    e2 = 100
    # Wait until the robot reaches the pose
    while (abs(e1) + abs(e2)) > tol:
        ja1 = my2DOFRobot.getJointAngle(MOTOR_ONE)
        sleep(0.05)
        ja2 = my2DOFRobot.getJointAngle(MOTOR_TWO)
        sleep(0.05)
        e1 = th1_vec[0] - ja1
        e2 = th2_vec[0] - ja2
        # print(f"joint angle errors: {e1:.3f}, {e2:.3f}")
        my2DOFRobot.setRobotState([[th1_vec[0], spd], [th2_vec[1], spd]])
        sleep(0.2)
    # Pen down
    # my2DOFRobot.penDown(90)
    #import wacom_trial
    #Run wacom_trial parallely to record pen data while tracking the trajectory
    subprocess.Popen([sys.executable, "wacom_trial.py"])
    sleep(2)


    # Open loop trajectory tracking
    # Track the straight line
    start_time = time.time()  # Record the start time
    next_time = start_time + dt  # Calculate the next time to send a command

    # Iterate over the length of th1 and th2 (which are the same length)
    idx=0
    for i in range(len(th1_vec)-1):
        # Set the robot state for each motor at each time step
        # my2DOFRobot.setRobotState([[th1_vec[i], th1_dot_vec[i]], [th2_vec[i], th2_dot_vec[i]]])
        fb = my2DOFRobot.setGetRobotState([[th1_vec[i], 1.5*th1_dot_vec[i]], [th2_vec[i], 1.5*th2_dot_vec[i]]])
        if fb != -1 and abs(th1_vec[i] - fb[0][0]) < 0.15 and abs(th2_vec[i] - fb[1][0]) < 0.15: # Check if feedback is valid and not an outlier
            [[th1_fb[idx], th1_dot_fb[idx]], [th2_fb[idx], th2_dot_fb[idx]]] = fb
            t_fb[idx] = time.time() - start_time
            idx+=1
        if time.time() < next_time:  # Wait until dt has elapsed
            print('idle time:',next_time-time.time())
            time.sleep(next_time - time.time())

        # Update the next time for the next command
        next_time += dt


    th1_fb = th1_fb[:idx]
    th2_fb = th2_fb[:idx]
    th1_dot_fb = th1_dot_fb[:idx]
    th2_dot_fb = th2_dot_fb[:idx]
    t_fb = t_fb[:idx]

    # sleep(1)
    # my2DOFRobot.penUp()
    # sleep(2)
    # my2DOFRobot.goHome()
    #save data
    trajdata=[x_vec,y_vec,t_vec,th1_vec,th2_vec,th1_dot_vec,th2_dot_vec,t_fb,th1_fb,th2_fb,th1_dot_fb,th2_dot_fb]
    pickle.dump(trajdata, open('traj'+ROBOT+'.pkl', 'wb'))
    #Save data as .dat files
    # save t_fb, th1_fb as .dat file    
    np.savetxt(os.path.join(dir,'th1act'+ROBOT+'.dat'), np.column_stack((t_fb, th1_fb)), delimiter='\t', header='Time\tth1_fb')
    # save t_fb, th2_fb as .dat file
    np.savetxt(os.path.join(dir,'th2act'+ROBOT+'.dat'), np.column_stack((t_fb, th2_fb)), delimiter='\t', header='Time\tth2_fb')

    # plot

    # Create a figure for 4 subplots
    plt.figure(figsize=(10, 8))

    # 1st subplot: th1 and th1_fb vs time
    plt.subplot(2, 2, 1)
    plt.plot(t_vec, th1_vec, label=r'$\theta_1$', color='b', marker='o', linestyle='-')
    # th_ind=th1_fb!=0
    plt.plot(t_fb , th1_fb , label=r'$\theta_{1_{act}}$', color='r', marker='x', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel(r'$\theta_1$ (rad)')
    # plt.title('th1 and th1_fb vs Time')
    plt.grid(True)
    plt.legend()

    # 2nd subplot: th2 and th2_fb vs time
    plt.subplot(2, 2, 2)
    plt.plot(t_vec, th2_vec, label=r'$\theta_2$', color='b', marker='o', linestyle='-')
    # th_ind=th2_fb!=0
    plt.plot(t_fb , th2_fb , label=r'$\theta_{2_{act}}$', color='r', marker='x', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel(r'$\theta_2$ (rad)')
    # plt.title('th2 and th2_fb vs Time')
    plt.grid(True)
    plt.legend()

    # 3rd subplot: th1_dot and th1_dot_fb vs time
    plt.subplot(2, 2, 3)
    plt.plot(t_vec[1:], th1_dot_vec, label='th1_dot', color='b', marker='s', linestyle='-')
    # dth_ind=th1_dot_fb!=0
    plt.plot(t_fb , th1_dot_fb , label='th1_dot_fb', color='r', marker='x', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel(r'$\dot{\theta}_1$ (rad/s)')
    # plt.title('th1_dot and th1_dot_fb vs Time')
    plt.grid(True)
    plt.legend()

    # 4th subplot: th2_dot and th2_dot_fb vs time
    plt.subplot(2, 2, 4)
    plt.plot(t_vec[1:], th2_dot_vec, label='th2_dot', color='b', marker='s', linestyle='-')
    # dth_ind=th2_dot_fb!=0
    plt.plot(t_fb , th2_dot_fb , label='th2_dot_fb', color='r', marker='x', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel(r'$\dot{\theta}_2$ (rad/s)')
    # plt.title('th2_dot and th2_dot_fb vs Time')
    plt.grid(True)
    plt.legend()

    # Adjust layout and show the plot
    plt.tight_layout()
    plt.show(block=False)

    # MuJoCo digital twin
    mj_sim(m,d,t_fb,th1_fb,th2_fb,xy_vec)

except:
    #Run mujoco simulation with the planned trajectory if robot operation fails
    mj_sim(m,d,t_vec,th1_vec,th2_vec,xy_vec)    
    # # trajdata=pickle.load(open(os.path.join(dir,'traj_5bar.pkl'), 'rb'))
    # # x_vec,y_vec=trajdata[0],trajdata[1]
    # # t_vec, th1_vec,th2_vec,t_vec,th1_dot_vec,th2_dot_vec = trajdata[2],trajdata[3],trajdata[4],trajdata[5],trajdata[6]
    # t_fb, th1_fb,th2_fb,th1_dot_fb,th2_dot_fb = trajdata[7],trajdata[8],trajdata[9],trajdata[10],trajdata[11]
    # #save data as .dat files
    # # Save t_fb, th1_fb as .dat file    
    # np.savetxt(os.path.join(dir,'th1actdata.dat'), np.column_stack((t_fb, th1_fb)), delimiter='\t', header='Time\tth1_fb')
    # # save t_fb, th2_fb as .dat file
    # np.savetxt(os.path.join(dir,'th2actdata.dat'), np.column_stack((t_fb, th2_fb)), delimiter='\t', header='Time\tth2_fb')

    # #Plan linear trajectory and track in MuJoCo
    # x1y1 = [0.25, -0.1]
    # x2y2 = [0.25, 0.1]
    # T = 1
    # th1_vec,th2_vec,t_vec,xy_vec =line_track(x1y1,x2y2,T)
    # mj_sim(m,d,t_vec,th1_vec,th2_vec,xy_vec,ti=1,isDone=False)

    # x1y1 = [0.25, 0.1]
    # x2y2 = [0.2, 0.1]
    # T = 1
    # th1_vec,th2_vec,t_vec,xy_vec =line_track(x1y1,x2y2,T)
    # mj_sim(m,d,t_vec,th1_vec,th2_vec,xy_vec,ti=0,isDone=True)    


