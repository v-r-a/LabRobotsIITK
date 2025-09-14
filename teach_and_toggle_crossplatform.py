from RobotAPI_class import RobotAPI
import time
from time import sleep
import sys

# -------------------------------------------------------------------------
# OS-specific keyboard handling
if sys.platform.startswith('win'):
    print("Windows OS detected. Using 'keyboard' library.")
    import keyboard

    def wait_for_key(target_key: str):
        """Wait until target_key is pressed (Windows)."""
        while True:
            key = keyboard.read_key()
            if key.lower() == target_key.lower():
                return

elif sys.platform.startswith('linux') or sys.platform.startswith('darwin'):
    print("Linux/macOS OS detected. Using 'pynput' library for keypress detection.")
    from pynput import keyboard as pkb

    def wait_for_key(target_key: str):
        """Wait until target_key is pressed (Linux/macOS)."""
        pressed = {"ok": False}

        def on_press(key):
            try:
                if key.char and key.char.lower() == target_key.lower():
                    pressed["ok"] = True
                    return False  # stop listener
            except AttributeError:
                if target_key == "enter" and key == pkb.Key.enter:
                    pressed["ok"] = True
                    return False

        with pkb.Listener(on_press=on_press) as listener:
            listener.join()
        return
else:
    raise RuntimeError("Unsupported OS for this script.")
# -------------------------------------------------------------------------

BAUDRATE = 1000000  # PC-Arduino communication rate
DEVICENAME = ("/dev/ttyUSB0")  # COM port number may vary
ROBOT = "2R"  # Robot name (used in some functions)
BASE_MOTOR = 1  # Predefined ID of the base motor
ELBOW_MOTOR = 2  # Predefined ID of the elbow motor

# Make an instance of RobotAPI class
my2R = RobotAPI(DEVICENAME, BAUDRATE, ROBOT)

# Robot pose variables
pose1 = [0, 0]
pose2 = [0, 0]
tol = 0.1  # joint angle tolerance (rad)

# -------------------------------------------------------------------------
def read_joint_angles(max_attempts=6, delay=0.1):
    """Try up to max_attempts to read both joints. Return [a0,a1] or None."""
    for _ in range(max_attempts):
        a0 = my2R.getJointAngle(BASE_MOTOR)
        sleep(delay)
        a1 = my2R.getJointAngle(ELBOW_MOTOR)
        sleep(delay)
        if a0 != -1 and a1 != -1:
            return [a0, a1]
    return None
# -------------------------------------------------------------------------

# Go to the home position
my2R.goHome()
print("Wait for the robot to complete homing...")
sleep(5)

# Turn off motor torques
op1 = op2 = False
i = 0
while not (op1 and op2):
    op1 = my2R.setTorqueOFF(BASE_MOTOR)
    sleep(0.2)
    op2 = my2R.setTorqueOFF(ELBOW_MOTOR)
    sleep(0.2)
    i += 1
    if i > 5:
        break

if op1 and op2:
    print("The motor torques have been turned off successfully.")
    print("The robot is free to move!")
else:
    print("Stop and Re-run the program. Not able to turn off either of the motor torques.")
    print("Press the RESET button on Arduino Mega")

# -------------------------------------------------------------------------
# Record position 1
print("Move the robot gently to the first desired location and then press 'r'")
wait_for_key('r')
angles = read_joint_angles()
if angles is None:
    print("Stop and re-run the program if you see Err: Timeout.")
else:
    pose1 = angles
    my2R.penDown(); sleep(0.5); my2R.penUp()
    print("Recorded the joint angles:", pose1)

# Record position 2
print("Move the robot gently to the second desired location and then press 'r'")
wait_for_key('r')
angles = read_joint_angles()
if angles is None:
    print("Stop and re-run the program if you see Err: Timeout.")
else:
    pose2 = angles
    my2R.penDown(); sleep(0.5); my2R.penUp()
    print("Recorded the joint angles:", pose2)

# -------------------------------------------------------------------------
# Start moving
print("Two robot positions recorded.")
print("Press 'Enter' and move aside!!!")
wait_for_key("enter")
sleep(1)

# Move to position 1
print("Moving to the first position...")
spd = 2
op = False
while not op:
    op = my2R.setRobotState([[pose1[0], spd], [pose1[1], spd]])
    if not op:
        print("Error in sending data, trying again.")

# Wait until the robot reaches the pose
e1 = e2 = 100
while (abs(e1) + abs(e2)) > tol:
    ja1 = my2R.getJointAngle(BASE_MOTOR); sleep(0.05)
    ja2 = my2R.getJointAngle(ELBOW_MOTOR); sleep(0.05)
    e1 = pose1[0] - ja1
    e2 = pose1[1] - ja2
    my2R.setRobotState([[pose1[0], spd], [pose1[1], spd]])
    sleep(0.2)

print("Achieved joint angles:", [ja1, ja2])
my2R.penDown(); sleep(0.5); my2R.penUp()

# Move to position 2
print("Moving to the second position...")
op = False
while not op:
    op = my2R.setRobotState([[pose2[0], spd], [pose2[1], spd]])
    if not op:
        print("Error in sending data, trying again.")

e1 = e2 = 100
while (abs(e1) + abs(e2)) > tol:
    ja1 = my2R.getJointAngle(BASE_MOTOR); sleep(0.05)
    ja2 = my2R.getJointAngle(ELBOW_MOTOR); sleep(0.05)
    e1 = pose2[0] - ja1
    e2 = pose2[1] - ja2
    my2R.setRobotState([[pose2[0], spd], [pose2[1], spd]])
    sleep(0.2)

print("Achieved joint angles:", [ja1, ja2])
my2R.penDown(); sleep(0.5); my2R.penUp()

# Finish
print("OK done. Homing now...")
op = False
i = 0
while not op:
    op = my2R.goHome()
    sleep(0.1)
    i += 1
    if i > 5:
        break

sleep(5)
print("End")
