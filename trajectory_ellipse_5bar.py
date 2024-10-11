from RobotAPI_class import RobotAPI
import time
from time import sleep
import keyboard
import numpy as np
import matplotlib.pyplot as plt
# Import the inverse kinematics functions
from IK_functions import *

# Robot architecture parameters
arch_param_5bar = [0.2,0.2,0.2,0.2,0.067]

#--------------------------------------------------------------------------------------------
# Offline trajectory planning (before sending it to the robot)

# Ellipse details
cen = [0.25, 0.1]
a = 0.04
b = 0.04

# Commands may be sent to the robot every 0.05 second (50 ms)
dt = 0.05

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
plt.show()  # Keep both windows open indefinitely

# Perform inverse kinematics for 5-bar
th1_th2_vec = np.array([IK_5bar(arch_param_5bar, x, y) for x, y in zip(x_vec, y_vec)])

th1_vec = th1_th2_vec[:, 0]
th2_vec = th1_th2_vec[:, 1]

# Calculate angular velocities using forward differences
th1_dot_vec = np.diff(th1_vec) / dt  # Angular velocity for th1_vec
th2_dot_vec = np.diff(th2_vec) / dt  # Angular velocity for th2_vec

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
plt.show()


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
plt.show()

#--------------------------------------------------------------------------------------------
# Robot operation
BAUDRATE = 1000000 # PC-Arduino communication rate
DEVICENAME = "COM9" # COM port number may vary
ROBOT = "5bar" # Robot name (used in some functions)
LEFT_MOTOR = 1 # Predefined ID of the LEFT motor
RIGHT_MOTOR = 2 # Predefined ID of the RIGHT motor

# Make an instance of RobotAPI class
my5bar = RobotAPI(DEVICENAME, BAUDRATE, ROBOT)

print("The joint angles and joint velocities are within limits, press any key to proceed,\n Else, interrupt the program operation.")
key = keyboard.read_key()

# Go to the start point and wait
print("Moving to the first position...")
spd = 1
op = False
while op == False:
    op = my5bar.setRobotState([[th1_vec[0],spd],[th1_vec[0],spd]])
    if op == False:
        print("Error in sending data, trying again...")

# Check feedback
tol = 0.05
e1 = 100
e2 = 100
# Wait until the robot reaches the pose
while (abs(e1) + abs(e2)) > tol:
    ja1 = my5bar.getJointAngle(LEFT_MOTOR)
    sleep(0.05)
    ja2 = my5bar.getJointAngle(RIGHT_MOTOR)
    sleep(0.05)
    e1 = th1_vec[0] - ja1
    e2 = th2_vec[0] - ja2
    # print(f"joint angle errors: {e1:.3f}, {e2:.3f}")
    my5bar.setRobotState([[th1_vec[0], spd], [th2_vec[1], spd]])
    sleep(0.2)
# Pen down
my5bar.penDown(90)

# Data to send to the motors: Next position, Avg speed to next position
th1_vec = th1_vec[1:]  # Remove the first element because the robot is already there
th2_vec = th2_vec[1:]  # Remove the first element because the robot is already there

# Store feedback angles
th1_fb = np.zeros(len(th1_vec))
th2_fb = np.zeros(len(th1_vec))
th1_dot_fb = np.zeros(len(th1_vec))
th2_dot_fb = np.zeros(len(th1_vec))
t_fb = np.zeros(len(th1_vec))

# Open loop trajectory tracking
# Track the straight line
start_time = time.time()  # Record the start time
next_time = start_time + dt  # Calculate the next time to send a command

# Iterate over the length of th1 and th2 (which are the same length)
for i in range(len(th1_vec)):
    while time.time() < next_time:  # Wait until dt has elapsed
        time.sleep(0.001)  # Sleep for 1 millisecond

    # Set the robot state for each motor at each time step
    # my5bar.setRobotState([[th1_vec[i], th1_dot_vec[i]], [th2_vec[i], th2_dot_vec[i]]])
    fb = my5bar.setGetRobotState([[th1_vec[i], th1_dot_vec[i]], [th2_vec[i], th2_dot_vec[i]]])
    if fb != -1:
        [[th1_fb[i], th1_dot_fb[i]], [th2_fb[i], th2_dot_fb[i]]] = fb
        t_fb[i] = time.time() - start_time
    # Update the next time for the next command
    next_time += dt

sleep(1)
my5bar.penUp()

# plot
import matplotlib.pyplot as plt

# Create a figure for 4 subplots
plt.figure(figsize=(10, 8))

# 1st subplot: th1 and th1_fb vs time
plt.subplot(2, 2, 1)
plt.plot(t_vec[1:], th1_vec, label='th1', color='b', marker='o', linestyle='-')
plt.plot(t_fb, th1_fb, label='th1_fb', color='b', marker='x', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('th1 (radians)')
plt.title('th1 and th1_fb vs Time')
plt.grid(True)
plt.legend()

# 2nd subplot: th2 and th2_fb vs time
plt.subplot(2, 2, 2)
plt.plot(t_vec[1:], th2_vec, label='th2', color='r', marker='o', linestyle='-')
plt.plot(t_fb, th2_fb, label='th2_fb', color='r', marker='x', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('th2 (radians)')
plt.title('th2 and th2_fb vs Time')
plt.grid(True)
plt.legend()

# 3rd subplot: th1_dot and th1_dot_fb vs time
plt.subplot(2, 2, 3)
plt.plot(t_vec[1:], th1_dot_vec, label='th1_dot', color='g', marker='s', linestyle='-')
plt.plot(t_fb, th1_dot_fb, label='th1_dot_fb', color='g', marker='x', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('th1_dot (rad/s)')
plt.title('th1_dot and th1_dot_fb vs Time')
plt.grid(True)
plt.legend()

# 4th subplot: th2_dot and th2_dot_fb vs time
plt.subplot(2, 2, 4)
plt.plot(t_vec[1:], th2_dot_vec, label='th2_dot', color='m', marker='s', linestyle='-')
plt.plot(t_fb, th2_dot_fb, label='th2_dot_fb', color='m', marker='x', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('th2_dot (rad/s)')
plt.title('th2_dot and th2_dot_fb vs Time')
plt.grid(True)
plt.legend()

# Adjust layout and show the plot
plt.tight_layout()
plt.show()


sleep(2)
my5bar.goHome()