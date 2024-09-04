# Important:
# USB Latency setting
# https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/#usb-latency-setting
# AX-12A motor
# https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
# For debugging, connect to Arduino Mega's HardWare Serial 1 UART port via USB-TTL converter

from RobotAPI_class import RobotAPI
import time
from time import sleep
import keyboard

BAUDRATE = 1000000 # PC-Arduino communication rate
DEVICENAME = "COM9" # COM port number may vary
ROBOT = "2R" # Robot name (used in some functions)
BASE_MOTOR = 1 # Predefined ID of the base motor
ELBOW_MOTOR = 2 # Predefined ID of the elbow motor

# Make an instance of RobotAPI class
my2R = RobotAPI(DEVICENAME, BAUDRATE, ROBOT)

# Robot pose variables
pose1 = [0,0]
pose2 = [0,0]
# joint angle tolerance (rad) in the target position
tol = 0.1

# Go to the home position
my2R.goHome()
print("Wait... the robot is homing")
sleep(5)

# Turn off the motor torques to allow manual movement
op1 = my2R.setTorqueOFF(BASE_MOTOR)
sleep(0.1)
op2 = my2R.setTorqueOFF(ELBOW_MOTOR)
sleep(0.1)
# Notify the user
if op1 and op2:
    print("The motor torques have been turned off successfully.")
    print("The robot is free to move!")
else:
    print("Stop and Re-run the program. Not able to turn off either of the motor torques.")
    print("Press the RESET button on Arduino MEGA")

#--------------------------------------------------------------------------------
# Record the position 1
print("Move the robot to the first desired location and then press 'R'")
key = keyboard.read_key()
if key.lower() == 'r':
    pose1[0] = my2R.getJointAngle(BASE_MOTOR)
    pose1[1] = my2R.getJointAngle(ELBOW_MOTOR)
    print("Recorded the pose. If you see Err: Timeout, stop and re-run the program.")
    my2R.penDown()
    sleep(0.5)
    my2R.penUp()
    print(pose1)

# Record the position 2
print("Move the robot to the second desired location and then press 'R'")
key = keyboard.read_key()
if key.lower() == 'r':
    pose2[0] = my2R.getJointAngle(BASE_MOTOR)
    pose2[1] = my2R.getJointAngle(ELBOW_MOTOR)
    print("Recorded the pose. If you see Err: Timeout, stop and re-run the program.")
    my2R.penDown()
    sleep(0.5)
    my2R.penUp()
    print(pose2)
#--------------------------------------------------------------------------------
# Start moving
print("Positions recorded. Press 'Enter' and move aside!!!")
key = keyboard.read_key()
sleep(1)
print("Moving to the first position...")
spd = 2
# set the target robot state: [[JA1,JV1],[JA2,JV2]], JA: joint angle, JV: joint velocity
op = False
while op == False:
    op = my2R.setRobotState([[pose1[0],spd],[pose1[1],spd]])
    if op == False:
        print("Error in sending data, trying again.")

# Check feedback
e1 = 100
e2 = 100
# Wait until the robot reaches the pose
while (abs(e1) + abs(e2)) > 0.05:
    ja1 = my2R.getJointAngle(BASE_MOTOR)
    sleep(0.05)
    ja2 = my2R.getJointAngle(ELBOW_MOTOR)
    sleep(0.05)
    e1 = pose1[0] - ja1
    e2 = pose1[1] - ja2
    print(f"joint angle errors: {e1:.3f}, {e2:.3f}")
    my2R.setRobotState([[pose1[0], spd], [pose1[1], spd]])
    sleep(0.2)

# Mark
sleep(0.5)
my2R.penDown()
sleep(0.5)
my2R.penUp()
sleep(1)

# Move to the second position
print("Moving to the the second position...")
# set the target robot state: [[JA1,JV1],[JA2,JV2]], JA: joint angle, JV: joint velocity
op = False
while op == False:
    op = my2R.setRobotState([[pose2[0],spd],[pose2[1],spd]])
    if op == False:
        print("Error in sending data, trying again.")

# Check feedback
e1 = 100
e2 = 100
# Wait until the robot reaches the pose
while (abs(e1) + abs(e2)) > 0.05:
    ja1 = my2R.getJointAngle(BASE_MOTOR)
    sleep(0.05)
    ja2 = my2R.getJointAngle(ELBOW_MOTOR)
    sleep(0.05)
    e1 = pose2[0] - ja1
    e2 = pose2[1] - ja2
    print(f"joint angle errors: {e1:.3f}, {e2:.3f}")
    my2R.setRobotState([[pose2[0], spd], [pose2[1], spd]])
    sleep(0.2)

# Mark
sleep(0.5)
my2R.penDown()
sleep(0.5)
my2R.penUp()
sleep(1)

# Finish
print("OK done. Homing now...")
my2R.goHome()
sleep(5)
print("End")