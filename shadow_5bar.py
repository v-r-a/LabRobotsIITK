from RobotAPI_class import RobotAPI
import time
from time import sleep
import mujoco
import mujoco.viewer
import numpy as np
import os

BAUDRATE = 1000000  # PC-Arduino communication rate
DEVICENAME = "COM9"  # COM port number may vary
ROBOT = "5bar"  # Robot name (used in some functions)
LEFT_MOTOR = 1  # Predefined ID of the LEFT motor
RIGHT_MOTOR = 2  # Predefined ID of the RIGHT motor

# Make an instance of RobotAPI class
my5bar = RobotAPI(DEVICENAME, BAUDRATE, ROBOT)

# Go home
my5bar.goHome()
sleep(3)

# Motor torque off
op = False
while op == False:
    op = my5bar.setTorqueOFF(LEFT_MOTOR)
    if op == 1:
        print("Torque off: LEFT_MOTOR")
    else:
        print("Failed to torque off: LEFT_MOTOR. Trying again...")
    op = my5bar.setTorqueOFF(RIGHT_MOTOR)
    if op == 1:
        print("Torque off: RIGHT_MOTOR")
    else:
        print("Failed to torque off: RIGHT_MOTOR. Trying again...")

sleep(1)

# MuJoCo model and simulation data initialisation
m = mujoco.MjModel.from_xml_path('os.getcwd()+'\\5bar.xml')
d = mujoco.MjData(m)

# Keyboard callback
pendown = False
draw_sph = False

def keyboard_func(keycode):
    if chr(keycode) == ' ':
        global pendown
        pendown = not pendown
    elif chr(keycode) == '1':
        global draw_sph
        draw_sph = not draw_sph
        

# Global variables for control
th1 = 0
th2 = 0
tol = 0.15

with mujoco.viewer.launch_passive(m, d, key_callback=keyboard_func) as viewer:
    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_ACTUATOR] = True
    # iterator for decorative geometry objects
    idx_geom = 0
    # Simulation loop
    while viewer.is_running():
        wallclock = time.time()
        simstart = d.time
        # Run the simulation for 1/60th of the second and then render
        th1_temp = my5bar.getJointAngle(LEFT_MOTOR)
        th2_temp = my5bar.getJointAngle(RIGHT_MOTOR)
        # Pass on only valid angles. Timeout => -1
        if th1_temp != -1 and th2_temp != -1:
            # Set the control inputs
            d.ctrl[0] = th1
            d.ctrl[1] = th2
        # Filter out the sudden jumps beyond the 'tol'
        if abs(th1-th1_temp) < tol:
            th1 = th1_temp
        if abs(th2-th2_temp) < tol:
            th2 = th2_temp

        while (d.time - simstart) < 1/60:
            mujoco.mj_step(m, d)


        # Pen up/down action
        if pendown:
            my5bar.penDown(90)
            draw_sph = 1
        else:
            my5bar.penUp()
            draw_sph = 0

        # simulated pen trace
        if draw_sph:
            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[idx_geom],
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=[0.003,0,0],
                pos = np.array([d.site_xpos[0][0],d.site_xpos[0][1],0]),
                mat=np.eye(3).flatten(),
                rgba=np.array([1,0,0,0.5])
            )
            idx_geom += 1
            viewer.user_scn.ngeom = idx_geom
            # Reset if the number of geometries hit the limit
            if idx_geom > (viewer.user_scn.maxgeom - 50):
                # Reset
                idx_geom = 1
        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        while (time.time() - wallclock) < 1/60:
            sleep((time.time() - wallclock)/4)
