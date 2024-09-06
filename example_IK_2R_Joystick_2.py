from RobotAPI_class import RobotAPI
import numpy as np
import time
from time import sleep
import pygame
import sys

# Import the inverse kinematics functions
from IK_functions import *

BAUDRATE = 1000000
DEVICENAME = "COM9"
ROBOT = "2R"
BASE_MOTOR = 1
ELBOW_MOTOR = 2

# Robot parameters
l1 = 0.2
l2 = 0.2

# End effector position
x = 0.399
y = 0.0

# Joint angles
a1 = 0.
a2 = 0.

# Estimated angles
th1e = 0
th2e = 1

my2R = RobotAPI(DEVICENAME, BAUDRATE, ROBOT)
sleep(1)

# Initialize Pygame
pygame.init()

# Initialize joystick
pygame.joystick.init()

# Ensure at least one joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick found.")
    sys.exit()

# Get the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Button mapping
button_a = 0 
button_b = 1
button_x = 2
button_y = 3
button_lb = 4
button_rb = 5
# Initialize counter
mode = 0
penState = 0

# Main loop
try:
    while True:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == button_rb:
                    penState = 1 - penState
                    if penState:
                        my2R.penDown()
                        sleep(0.1)
                    else:
                        my2R.penUp()
                        sleep(0.1)
                        
                if event.button == button_lb:
                    # Toggle between joint mode (0) and cartesian mode (1)
                    mode = 1 - mode
                    print(mode)
                if event.button == button_x:
                    if mode:
                        # Increment the X coordinate by -5 mm
                        x -= 0.005
                    else:
                        # Increment the base motor angle by -5 deg
                        a1 -= 5
                elif event.button == button_y:
                    if mode:
                        # Increment the X coordinate by +5 mm
                        x += 0.005
                    else:
                        # Increment the base motor angle by +5 deg
                        a1 += 5
                elif event.button == button_a:
                    if mode:
                        # Increment the Y coordinate by -5 mm
                        y-= 0.005
                    else:
                        # Increment the elbow motor angle by -5 deg
                        a2-= 5
                elif event.button == button_b:
                    if mode:
                        # Increment the Y coordinate by +5 mm
                        y += 0.005
                    else:
                        # Increment the elbow motor angle by +5 deg
                        a2 += 5
                if mode:
                    # Solve the IK
                    sol = IK_2R(l1, l2, x, y, 0, 1, "rad", "deg")
                    # Move the robot to the first solution
                    my2R.setJointAngle(BASE_MOTOR, sol[0][0], "deg")
                    my2R.setJointAngle(ELBOW_MOTOR, sol[0][1], "deg")
                else:
                    # Move the robot to the first solution
                    my2R.setJointAngle(BASE_MOTOR, a1, "deg")
                    my2R.setJointAngle(ELBOW_MOTOR, a2, "deg")
        # Delay to avoid busy-waiting
        pygame.time.wait(50)

except KeyboardInterrupt:
    pass

finally:
    # Quit Pygame
    pygame.quit()
