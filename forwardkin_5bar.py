from RobotAPI_class import RobotAPI
import time
from time import sleep
import mujoco
import mujoco.viewer
import numpy as np
import os

BAUDRATE = 1000000 # PC-Arduino communication rate
DEVICENAME = "COM9" # COM port number may vary
ROBOT = "5bar" # Robot name (used in some functions)
LEFT_MOTOR = 1 # Predefined ID of the LEFT motor
RIGHT_MOTOR = 2 # Predefined ID of the RIGHT motor

# Make an instance of RobotAPI class
my5bar = RobotAPI(DEVICENAME, BAUDRATE, ROBOT)

# MuJoCo model and simulation data initialisation
m = mujoco.MjModel.from_xml_path(os.getcwd()+'\\5bar.xml')
d = mujoco.MjData(m)

# Keyboard callback
pendown = False
def keyboard_func(keycode):
  if chr(keycode) == ' ':
    global pendown
    pendown = not pendown



with mujoco.viewer.launch_passive(m, d,key_callback=keyboard_func) as viewer:
  while viewer.is_running():
    step_start = time.time()

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)

    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_ACTUATOR] = True

      # Copy the pen site x-y coordinates
      end_effector_xy = d.site_xpos[0][0:2]
      # Make a copy and convert to millimeters
      end_effector_xy_mm = np.round(np.copy(end_effector_xy) * 1000, 0)
      print("penXY:", end_effector_xy_mm)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    if pendown:
      my5bar.penDown(90)
    else:
      my5bar.penUp()
    
    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)

    # Operate the robot at ~60 Hz
    my5bar.setJointAngle(LEFT_MOTOR, d.ctrl[0])
    my5bar.setJointAngle(RIGHT_MOTOR, d.ctrl[1])
