# Python API for robot control.

import math
import time
from time import sleep
from pySerialTransfer import pySerialTransfer as txfer


class RobotAPI:

    # Constants: Address
    ADDR_PC = 100
    ADDR_MEGA = 200
    # Constants: Function Code
    FC_PING = 1
    FC_TORQUE_ON = 2
    FC_TORQUE_OFF = 3
    FC_TORQUE_STATUS = 4
    FC_READ_ANGLE = 5
    FC_WRITE_ANGLE = 6
    FC_PEN_SERVO = 7
    FC_READ_STATE = 8
    FC_WRITE_STATE = 9
    FC_READ_ROBOT_STATE = 10
    FC_WRITE_ROBOT_STATE = 11
    FC_RW_ROBOT_STATE = 12
    # Other
    RS485_TO = 0.2

    # The constructor
    def __init__(self, port: str, baud_rate: int, robot_type: str):
        self.link = txfer.SerialTransfer(port, baud_rate)
        # Open the serial port, and if it fails, print an error message
        if self.link.open():
            print("Serial port opened successfully.")
        else:
            print("Failed to open serial port.")
        sleep(0.1)
        self.rob = robot_type

    # The destructor
    def __del__(self):
        self.link.close()

    def pingTest(self, motorID: int):
        """
        Description: This function sends a ping command
        to the specified motor and waits for a response.

        Input:
        motorID: The ID of the motor to ping

        Output:
        True if the motor responds to the ping command
        False if the motor does not respond to the ping command

        """

        # Packet to send
        from_addr = self.ADDR_PC
        to_addr = self.ADDR_MEGA
        FC1 = self.FC_PING
        FC2 = 0
        data = motorID
        # Packet
        my_array = list()
        my_array.append(from_addr)
        my_array.append(to_addr)
        my_array.append(FC1)
        my_array.append(FC2)
        my_array.append(data)

        try:
            # Load the Tx buffer with the packet
            send_size = self.self.link.tx_obj(my_array)
            # Send the packet
            self.self.link.send(send_size)
            # Wait for the packet to be sent
            sleep(0.001)
            # Record the time the packet was sent
            send_time = time.time()

            # Wait for a response
            data_recieved = True
            while not self.self.link.available():
                if self.link.status < 0:
                    if self.link.status == txfer.CRC_ERROR:
                        print("ERROR: CRC_ERROR")
                    elif self.link.status == txfer.PAYLOAD_ERROR:
                        print("ERROR: PAYLOAD_ERROR")
                    elif self.link.status == txfer.STOP_BYTE_ERROR:
                        print("ERROR: STOP_BYTE_ERROR")
                    else:
                        print("ERROR: {}".format(self.link.status))
                    return False
                sleep(0.001)
                # Check for time out
                if (time.time() - send_time) > self.RS485_TO:
                    print("Err: Time out")
                    data_recieved = False
                    return False

            if data_recieved:
                # print(f"ByteRead: {link.bytesRead}")
                # Process the response if needed
                response = self.self.link.rx_obj(
                    obj_type=list, obj_byte_size=self.link.bytesRead, list_format="i"
                )
                from_addr = response[0]
                to_addr = response[1]
                FC1 = response[2]
                FC2 = response[3]
                if (
                    from_addr == self.ADDR_MEGA
                    and to_addr == self.ADDR_PC
                    and FC1 == 0
                    and FC2 == self.FC_PING
                ):
                    return response[4]
                else:
                    print(
                        f"Error: Invalid response from {from_addr} to {to_addr} with FC1={FC1} and FC2={FC2}"
                    )
                    return False

        except Exception as e:
            print("Error:", e)
            return False

        return False

    def setTorqueON(self, motorID: int):
        """
        Description: This function enables the motor control
        of the specified motor and waits for a response.

        Input:
        motorID: The ID of the motor to enable torque

        Output:
        True if the torque is enabled
        False if failed to enabled the torque

        """

        # Packet to send
        from_addr = self.ADDR_PC
        to_addr = self.ADDR_MEGA
        FC1 = self.FC_TORQUE_ON
        FC2 = 0
        data = motorID
        # Packet
        my_array = list()
        my_array.append(from_addr)
        my_array.append(to_addr)
        my_array.append(FC1)
        my_array.append(FC2)
        my_array.append(data)

        try:
            # Load the Tx buffer with the packet
            send_size = self.self.link.tx_obj(my_array)
            # Send the packet
            self.link.send(send_size)
            # Wait for the packet to be sent
            sleep(0.001)
            # Record the time the packet was sent
            send_time = time.time()

            # Wait for a response
            data_recieved = True
            while not self.link.available():
                if self.link.status < 0:
                    if self.link.status == txfer.CRC_ERROR:
                        print("ERROR: CRC_ERROR")
                    elif self.link.status == txfer.PAYLOAD_ERROR:
                        print("ERROR: PAYLOAD_ERROR")
                    elif self.link.status == txfer.STOP_BYTE_ERROR:
                        print("ERROR: STOP_BYTE_ERROR")
                    else:
                        print("ERROR: {}".format(self.link.status))
                    return False
                sleep(0.001)
                # Check for time out
                if (time.time() - send_time) > self.RS485_TO:
                    print("Err: Time out")
                    data_recieved = False
                    return False

            if data_recieved:
                # print(f"ByteRead: {link.bytesRead}")
                # Process the response if needed
                response = self.link.rx_obj(
                    obj_type=list, obj_byte_size=self.link.bytesRead, list_format="i"
                )
                from_addr = response[0]
                to_addr = response[1]
                FC1 = response[2]
                FC2 = response[3]
                if (
                    from_addr == self.ADDR_MEGA
                    and to_addr == self.ADDR_PC
                    and FC1 == 0
                    and FC2 == self.FC_TORQUE_ON
                ):
                    return response[4]
                else:
                    print(
                        f"Error: Invalid response from {from_addr} to {to_addr} with FC1={FC1} and FC2={FC2}"
                    )
                    return False

        except Exception as e:
            print("Error:", e)
            return False

        return False

    def setTorqueOFF(self, motorID: int):
        """
        Description: This function disables the motor control
        of the specified motor and waits for a response.

        Input:
        motorID: The ID of the motor to disable torque

        Output:
        True if the torque is disabled
        False if failed to disable the torque

        """

        # Packet to send
        from_addr = self.ADDR_PC
        to_addr = self.ADDR_MEGA
        FC1 = self.FC_TORQUE_OFF
        FC2 = 0
        data = motorID
        # Packet
        my_array = list()
        my_array.append(from_addr)
        my_array.append(to_addr)
        my_array.append(FC1)
        my_array.append(FC2)
        my_array.append(data)

        try:
            # Load the Tx buffer with the packet
            send_size = self.link.tx_obj(my_array)
            # Send the packet
            self.link.send(send_size)
            # Wait for the packet to be sent
            sleep(0.001)
            # Record the time the packet was sent
            send_time = time.time()

            # Wait for a response
            data_recieved = True
            while not self.link.available():
                if self.link.status < 0:
                    if self.link.status == txfer.CRC_ERROR:
                        print("ERROR: CRC_ERROR")
                    elif self.link.status == txfer.PAYLOAD_ERROR:
                        print("ERROR: PAYLOAD_ERROR")
                    elif self.link.status == txfer.STOP_BYTE_ERROR:
                        print("ERROR: STOP_BYTE_ERROR")
                    else:
                        print("ERROR: {}".format(self.link.status))
                    return False
                sleep(0.001)
                # Check for time out
                if (time.time() - send_time) > self.RS485_TO:
                    print("Error: Time out")
                    data_recieved = False
                    return False

            if data_recieved:
                # print(f"ByteRead: {self.link.bytesRead}")
                # Process the response if needed
                response = self.link.rx_obj(
                    obj_type=list, obj_byte_size=self.link.bytesRead, list_format="i"
                )
                from_addr = response[0]
                to_addr = response[1]
                FC1 = response[2]
                FC2 = response[3]
                if (
                    from_addr == self.ADDR_MEGA
                    and to_addr == self.ADDR_PC
                    and FC1 == 0
                    and FC2 == self.FC_TORQUE_OFF
                ):
                    return response[4]
                else:
                    print(
                        f"Err: Invalid response from {from_addr} to {to_addr} with FC1={FC1} and FC2={FC2}"
                    )
                    return False

        except Exception as e:
            print("Error:", e)
            return False

        return False

    def getTorqueStatus(self, motorID: int):
        """
        Description: This function checks the torque enabled/disabled status.

        Input:
        motorID: The ID of the motor to check the torque status

        Output:
        1 if the motor torque is enabled
        0 if the motor torque is disabled
        -1 if failed to get the torque status

        """

        # Packet to send
        from_addr = self.ADDR_PC
        to_addr = self.ADDR_MEGA
        FC1 = self.FC_TORQUE_STATUS
        FC2 = 0
        data = motorID
        # Packet
        my_array = list()
        my_array.append(from_addr)
        my_array.append(to_addr)
        my_array.append(FC1)
        my_array.append(FC2)
        my_array.append(data)

        try:
            # Load the Tx buffer with the packet
            send_size = self.link.tx_obj(my_array)
            # Send the packet
            self.link.send(send_size)
            # Wait for the packet to be sent
            sleep(0.001)
            # Record the time the packet was sent
            send_time = time.time()

            # Wait for a response
            data_recieved = True
            while not self.link.available():
                if self.link.status < 0:
                    if self.link.status == txfer.CRC_ERROR:
                        print("ERROR: CRC_ERROR")
                    elif self.link.status == txfer.PAYLOAD_ERROR:
                        print("ERROR: PAYLOAD_ERROR")
                    elif self.link.status == txfer.STOP_BYTE_ERROR:
                        print("ERROR: STOP_BYTE_ERROR")
                    else:
                        print("ERROR: {}".format(self.link.status))
                    return False
                sleep(0.001)
                # Check for time out
                if (time.time() - send_time) > self.RS485_TO:
                    print("Err: Time out")
                    data_recieved = False
                    return -1

            if data_recieved:
                # print(f"ByteRead: {self.link.bytesRead}")
                # Process the response if needed
                response = self.link.rx_obj(
                    obj_type=list, obj_byte_size=self.link.bytesRead, list_format="i"
                )
                from_addr = response[0]
                to_addr = response[1]
                FC1 = response[2]
                FC2 = response[3]
                if (
                    from_addr == self.ADDR_MEGA
                    and to_addr == self.ADDR_PC
                    and FC1 == 0
                    and FC2 == self.FC_TORQUE_STATUS
                ):
                    return response[4]
                else:
                    print(
                        f"Error: Invalid response from {from_addr} to {to_addr} with FC1={FC1} and FC2={FC2}"
                    )
                    return -1

        except Exception as e:
            print("Error:", e)
            return -1

        return -1

    def _clip(self, value: int, min_value: int, max_value: int):
        """
        Clips the input value to the specified range.

        Parameters:
        value (int): The input value to be clipped.
        min_value (int): The minimum value of the range.
        max_value (int): The maximum value of the range.

        Returns:
        int: The clipped value.
        """
        op = max(min_value, min(value, max_value))
        if op != value:
            print(f"Warning: Value {value} to be sent to the robot clipped to {op}")
        return op

    def _mapAngleToRobot(
        self, robot: str, motorID: int, angle: float, unit: str = "rad"
    ):
        """
        Description:
        This function maps the angle in radians to a 10-bit value.
        The home (zero) position and the joint limits are preset
        according to the motor ID and the robot type.

        Input:
        robot: '2R' or '5bar'
        motorID: The ID of the motor
        angle: The angle in radians.
        unit: The unit of the angle. Default is 'rad'.  Use 'deg' for degrees.

        Output:
        10-bit value of the angle to be communicated to the robot
        -1: invalid robot type or unit
        """

        # base motor ID = 1, elbow motor ID = 2.
        # home at centre position for both the motors
        home_2R = [512, 512]
        # base motor ID 1 limits: + - 90 degrees, i.e., 307 counts
        # elbow motor ID 2 limits: + - 144 degrees, i.e., 490 counts
        limits_2R = [[512 - 307, 512 + 307], [512 - 490, 512 + 490]]

        # left motor ID = 1, right motor ID = 2.
        # home at 45 degrees away from the centre position for both the motors
        home_5bar = [512 - 153, 512 + 153]
        # left motor ID 1 limits: + 45 degrees and -90 degrees
        # right motor ID 2 limits: + 90 degrees and -45 degrees
        limits_5bar = [[512 - 307, 512 + 153], [512 - 153, 512 + 307]]

        # scaling factor
        if not isinstance(unit, str):
            raise TypeError("Error: unit input should be a string")
        else:
            if unit == "rad":
                scl = -(180 / math.pi) * (1023 / 300)
            elif unit == "deg":
                scl = -1023 / 300
            else:
                print(
                    "Error: Invalid unit entered. Please enter either 'rad' or 'deg' only"
                )
                return -1

        # check if the robot input is a string
        if not isinstance(robot, str):
            raise TypeError("Error: robot input should be a string")
        else:
            # if the robot is 2R
            if robot == "2R":
                # check if the motor ID is 1 or 2
                if motorID == 1:
                    # map the angle to the 10-bit value
                    angle_10bit = int(scl * angle + home_2R[0])
                    angle_10bit = self._clip(
                        angle_10bit, limits_2R[0][0], limits_2R[0][1]
                    )
                elif motorID == 2:
                    # map the angle to the 10-bit value
                    angle_10bit = int(scl * angle + home_2R[1])
                    angle_10bit = self._clip(
                        angle_10bit, limits_2R[1][0], limits_2R[1][1]
                    )
            elif robot == "5bar":
                # check if the motor ID is 1 or 2
                if motorID == 1:
                    # map the angle to the 10-bit value
                    angle_10bit = int(scl * angle + home_5bar[0])
                    angle_10bit = self._clip(
                        angle_10bit, limits_5bar[0][0], limits_5bar[0][1]
                    )
                elif motorID == 2:
                    # map the angle to the 10-bit value
                    angle_10bit = int(scl * angle + home_5bar[1])
                    angle_10bit = self._clip(
                        angle_10bit, limits_5bar[1][0], limits_5bar[1][1]
                    )
            else:
                print(
                    "Error: Invalid robot type entered. Please enter either '2R' or '5bar' only"
                )
                return -1
            return angle_10bit

    def _mapAngleFromRobot(
        self, robot: str, motorID: int, angle_10bit: int, unit: str = "rad"
    ):
        """
        Description:
        This function maps the 10-bit motor angle to the robot angle in radians.
        The home (zero) position and the joint limits are preset
        according to the motor ID and the robot type.

        Input:
        robot: '2R' or '5bar'
        motorID: The ID of the motor
        angle: The angle in radians.
        unit: The unit of the output angle. Default is 'rad'.  Use 'deg' for degrees.

        Output:
        The robot angle
        -1: invalid robot type or unit
        """

        # base motor ID = 1, elbow motor ID = 2.
        # home at centre position for both the motors
        home_2R = [512, 512]
        # base motor ID 1 limits: + - 90 degrees, i.e., 307 counts
        # elbow motor ID 2 limits: + - 144 degrees, i.e., 490 counts
        limits_2R = [[512 - 307, 512 + 307], [512 - 490, 512 + 490]]

        # left motor ID = 1, right motor ID = 2.
        # home at 45 degrees away from the centre position for both the motors
        home_5bar = [512 - 153, 512 + 153]
        # left motor ID 1 limits: + 45 degrees and -90 degrees
        # right motor ID 2 limits: + 90 degrees and -45 degrees
        limits_5bar = [[512 - 307, 512 + 153], [512 - 153, 512 + 307]]

        # scaling factor
        if not isinstance(unit, str):
            raise TypeError("Error: unit input should be a string")
        else:
            if unit == "rad":
                scl = -1 / ((180 / math.pi) * (1023 / 300))
            elif unit == "deg":
                scl = -1 / (1023 / 300)
            else:
                print(
                    "Error: Invalid unit entered. Please enter either 'rad' or 'deg' only"
                )
                return -1

        # check if the robot input is a string
        if not isinstance(robot, str):
            raise TypeError("Error: robot input should be a string")
        else:
            # if the robot is 2R
            if robot == "2R":
                # check if the motor ID is 1 or 2
                if motorID == 1:
                    # map the angle to the 16-bit value
                    angle = scl * (angle_10bit - home_2R[0])
                elif motorID == 2:
                    # map the angle to the 16-bit value
                    angle = scl * (angle_10bit - home_2R[1])
            elif robot == "5bar":
                # check if the motor ID is 1 or 2
                if motorID == 1:
                    # map the angle to the 16-bit value
                    angle = scl * (angle_10bit - home_5bar[0])
                elif motorID == 2:
                    angle = scl * (angle_10bit - home_5bar[1])
            else:
                print(
                    "Error: Invalid robot type entered. Please enter either '2R' or '5bar' only"
                )
                return -1
            return round(angle, 3)

    def getJointAngle(self, motorID: int, unit: str = "rad"):
        """
        Description: This function reads the motor joint angle

        Input:
        motorID: The ID of the motor
        unit: optional input. Default is 'rad'. Use 'deg' for degrees.

        Output:
        joint angle
        -1 if failed to get the joint angle

        """

        # Packet to send
        from_addr = self.ADDR_PC
        to_addr = self.ADDR_MEGA
        FC1 = self.FC_READ_ANGLE
        FC2 = 0
        data = motorID
        # Packet
        my_array = list()
        my_array.append(from_addr)
        my_array.append(to_addr)
        my_array.append(FC1)
        my_array.append(FC2)
        my_array.append(data)

        try:
            # Load the Tx buffer with the packet
            send_size = self.link.tx_obj(my_array)
            # Send the packet
            self.link.send(send_size)
            # Wait for the packet to be sent
            sleep(0.001)
            # Record the time the packet was sent
            send_time = time.time()

            # Wait for a response
            data_recieved = True
            while not self.link.available():
                if self.link.status < 0:
                    if self.link.status == txfer.CRC_ERROR:
                        print("ERROR: CRC_ERROR")
                    elif self.link.status == txfer.PAYLOAD_ERROR:
                        print("ERROR: PAYLOAD_ERROR")
                    elif self.link.status == txfer.STOP_BYTE_ERROR:
                        print("ERROR: STOP_BYTE_ERROR")
                    else:
                        print("ERROR: {}".format(self.link.status))
                    return False
                sleep(0.001)
                # Check for time out
                if (time.time() - send_time) > self.RS485_TO:
                    print("Err: Time out")
                    data_recieved = False
                    return -1

            if data_recieved:
                # print(f"ByteRead: {self.link.bytesRead}")
                # Process the response if needed
                response = self.link.rx_obj(
                    obj_type=list, obj_byte_size=self.link.bytesRead, list_format="i"
                )
                from_addr = response[0]
                to_addr = response[1]
                FC1 = response[2]
                FC2 = response[3]

                if (
                    from_addr == self.ADDR_MEGA
                    and to_addr == self.ADDR_PC
                    and FC1 == 0
                    and FC2 == self.FC_READ_ANGLE
                ):
                    angle = self._mapAngleFromRobot(
                        self.rob, motorID, response[4], unit
                    )
                    return angle
                else:
                    print(
                        f"Error: Invalid response from {from_addr} to {to_addr} with FC1={FC1} and FC2={FC2}"
                    )
                    return -1

        except Exception as e:
            print("Error:", e)
            return -1

        return -1

    def setJointAngle(
        self,
        motorID: int,
        angle: float,
        unit: str = "rad",
    ):
        """
        Description:
        This function sets the motor joint angle.
        The motor torque is enabled automatically.

        Input:
        motorID: The ID of the motor
        angle: The angle
        unit: optional input. Default is 'rad'. Use 'deg' for degrees.

        Output:
        success/failure

        """

        # Packet to send
        from_addr = self.ADDR_PC
        to_addr = self.ADDR_MEGA
        FC1 = self.FC_WRITE_ANGLE
        FC2 = 0
        # Packet
        my_array = list()
        my_array.append(from_addr)
        my_array.append(to_addr)
        my_array.append(FC1)
        my_array.append(FC2)
        my_array.append(motorID)
        motorAngle = self._mapAngleToRobot(self.rob, motorID, angle, unit)
        my_array.append(motorAngle)

        try:
            # Load the Tx buffer with the packet
            send_size = self.link.tx_obj(my_array)
            # Send the packet
            self.link.send(send_size)
            # Wait for the packet to be sent
            sleep(0.001)
            # Record the time the packet was sent
            send_time = time.time()

            # Wait for a response
            data_recieved = True
            while not self.link.available():
                if self.link.status < 0:
                    if self.link.status == txfer.CRC_ERROR:
                        print("ERROR: CRC_ERROR")
                    elif self.link.status == txfer.PAYLOAD_ERROR:
                        print("ERROR: PAYLOAD_ERROR")
                    elif self.link.status == txfer.STOP_BYTE_ERROR:
                        print("ERROR: STOP_BYTE_ERROR")
                    else:
                        print("ERROR: {}".format(self.link.status))
                    return False
                sleep(0.001)
                # Check for time out
                if (time.time() - send_time) > self.RS485_TO:
                    print("Err: Time out")
                    data_recieved = False
                    return -1

            if data_recieved:
                # print(f"ByteRead: {self.link.bytesRead}")
                # Process the response if needed
                response = self.link.rx_obj(
                    obj_type=list, obj_byte_size=self.link.bytesRead, list_format="i"
                )
                from_addr = response[0]
                to_addr = response[1]
                FC1 = response[2]
                FC2 = response[3]

                if (
                    from_addr == self.ADDR_MEGA
                    and to_addr == self.ADDR_PC
                    and FC1 == 0
                    and FC2 == self.FC_WRITE_ANGLE
                ):
                    # Read the acknowledgement
                    ack = response[4]
                    return ack
                else:
                    print(
                        f"Error: Invalid response from {from_addr} to {to_addr} with FC1={FC1} and FC2={FC2}"
                    )
                    return -1

        except Exception as e:
            print("Error:", e)
            return -1

        return -1

    def penDown(self):
        """
        Description:
        Pen down

        Input: -NA-

        Output:
        No feedback available from the servo motor. Inspect visually.

        """

        # Packet to send
        from_addr = self.ADDR_PC
        to_addr = self.ADDR_MEGA
        FC1 = self.FC_PEN_SERVO
        FC2 = 0
        data = 30  # pen down => move servo to 30 degrees
        # Packet
        my_array = list()
        my_array.append(from_addr)
        my_array.append(to_addr)
        my_array.append(FC1)
        my_array.append(FC2)
        my_array.append(data)

        try:
            # Load the Tx buffer with the packet
            send_size = self.link.tx_obj(my_array)
            # Send the packet
            self.link.send(send_size)
            # Wait for the packet to be sent
            sleep(0.001)
            # Record the time the packet was sent
            send_time = time.time()

            # Wait for a response
            data_recieved = True
            while not self.link.available():
                if self.link.status < 0:
                    if self.link.status == txfer.CRC_ERROR:
                        print("ERROR: CRC_ERROR")
                    elif self.link.status == txfer.PAYLOAD_ERROR:
                        print("ERROR: PAYLOAD_ERROR")
                    elif self.link.status == txfer.STOP_BYTE_ERROR:
                        print("ERROR: STOP_BYTE_ERROR")
                    else:
                        print("ERROR: {}".format(self.link.status))
                    return False
                sleep(0.001)

                if (time.time() - send_time) > self.RS485_TO:
                    print("Err: Time out")
                    data_recieved = False
                    return False

            if data_recieved:
                # print(f"ByteRead: {self.link.bytesRead}")
                # Process the response if needed
                response = self.link.rx_obj(
                    obj_type=list, obj_byte_size=self.link.bytesRead, list_format="i"
                )
                from_addr = response[0]
                to_addr = response[1]
                FC1 = response[2]
                FC2 = response[3]
                if (
                    from_addr == self.ADDR_MEGA
                    and to_addr == self.ADDR_PC
                    and FC1 == 0
                    and FC2 == self.FC_PEN_SERVO
                ):
                    data = response[4]
                    return data
                else:
                    print(
                        f"Error: Invalid response from {from_addr} to {to_addr} with FC1={FC1} and FC2={FC2}"
                    )
                    return False

        except Exception as e:
            print("Error:", e)
            return False

        return False

    def penUp(self):
        """
        Description:
        Pen up (lifted in air)

        Input: -NA-

        Output:
        No feedback available from the servo motor. Inspect visually.

        """

        # Packet to send
        from_addr = self.ADDR_PC
        to_addr = self.ADDR_MEGA
        FC1 = self.FC_PEN_SERVO
        FC2 = 0
        data = 0  # pen up => move servo to 0 degrees
        # Packet
        my_array = list()
        my_array.append(from_addr)
        my_array.append(to_addr)
        my_array.append(FC1)
        my_array.append(FC2)
        my_array.append(data)

        try:
            # Load the Tx buffer with the packet
            send_size = self.link.tx_obj(my_array)
            # Send the packet
            self.link.send(send_size)
            # Wait for the packet to be sent
            sleep(0.001)
            # Record the time the packet was sent
            send_time = time.time()

            # Wait for a response
            data_recieved = True
            while not self.link.available():
                if self.link.status < 0:
                    if self.link.status == txfer.CRC_ERROR:
                        print("ERROR: CRC_ERROR")
                    elif self.link.status == txfer.PAYLOAD_ERROR:
                        print("ERROR: PAYLOAD_ERROR")
                    elif self.link.status == txfer.STOP_BYTE_ERROR:
                        print("ERROR: STOP_BYTE_ERROR")
                    else:
                        print("ERROR: {}".format(self.link.status))
                    return False
                sleep(0.001)
                # Check for time out
                if (time.time() - send_time) > self.RS485_TO:
                    print("Err: Time out")
                    data_recieved = False
                    return False

            if data_recieved:
                # print(f"ByteRead: {self.link.bytesRead}")
                # Process the response if needed
                response = self.link.rx_obj(
                    obj_type=list, obj_byte_size=self.link.bytesRead, list_format="i"
                )
                from_addr = response[0]
                to_addr = response[1]
                FC1 = response[2]
                FC2 = response[3]
                if (
                    from_addr == self.ADDR_MEGA
                    and to_addr == self.ADDR_PC
                    and FC1 == 0
                    and FC2 == self.FC_PEN_SERVO
                ):
                    data = response[4]
                    return data
                else:
                    print(
                        f"Error: Invalid response from {from_addr} to {to_addr} with FC1={FC1} and FC2={FC2}"
                    )
                    return False

        except Exception as e:
            print("Error:", e)
            return False

        return False

    def _mapAngVelFromMotor(self, val_10bit: int):
        """
        Description:
        This function maps the 10-bit motor speed to the angular velocity in rad/s.
        Angular velocity mapping to/from robot is independent of the robot type and motor ID.

        Input:
        val_10bit: The 10-bit motor speed value.

        Output:
        The angular velocity in rad/s with sign.
        """

        # Present speed can be positive or negative like angular velocity.
        # AX-12A: 0-1023 is CCW, 1024-2047 is CW rotation
        # See https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#present-speed
        # Output in rad/s with sign.

        scl = -1 * (114 / 1023) * (2 * math.pi / 60)

        if val_10bit > 1023:
            op = -(val_10bit - 1024) * scl
            op = round(op, 3)
            return op
        else:
            op = val_10bit * scl
            op = round(op, 3)
            return op

    def getJointState(self, motorID: int):
        """
        Description: This function reads the motor joint angle and joint velocity

        Input:
        motorID: The ID of the motor

        Output:
        [joint angle (rad), joint velocity (rad/s)]
        -1 if failed to get the state

        """

        # Packet to send
        from_addr = self.ADDR_PC
        to_addr = self.ADDR_MEGA
        FC1 = self.FC_READ_STATE
        FC2 = 0
        # Packet
        my_array = list()
        my_array.append(from_addr)
        my_array.append(to_addr)
        my_array.append(FC1)
        my_array.append(FC2)
        my_array.append(motorID)

        try:
            # Load the Tx buffer with the packet
            send_size = self.link.tx_obj(my_array)
            # Send the packet
            self.link.send(send_size)
            # Wait for the packet to be sent
            sleep(0.001)
            # Record the time the packet was sent
            send_time = time.time()

            # Wait for a response
            data_recieved = True
            while not self.link.available():
                if self.link.status < 0:
                    if self.link.status == txfer.CRC_ERROR:
                        print("ERROR: CRC_ERROR")
                    elif self.link.status == txfer.PAYLOAD_ERROR:
                        print("ERROR: PAYLOAD_ERROR")
                    elif self.link.status == txfer.STOP_BYTE_ERROR:
                        print("ERROR: STOP_BYTE_ERROR")
                    else:
                        print("ERROR: {}".format(self.link.status))
                    return False
                sleep(0.001)
                # Check for time out
                if (time.time() - send_time) > self.RS485_TO:
                    print("Err: Time out")
                    data_recieved = False
                    return -1

            if data_recieved:
                # print(f"ByteRead: {self.link.bytesRead}")
                # Process the response if needed
                response = self.link.rx_obj(
                    obj_type=list, obj_byte_size=self.link.bytesRead, list_format="i"
                )
                from_addr = response[0]
                to_addr = response[1]
                FC1 = response[2]
                FC2 = response[3]
                if (
                    from_addr == self.ADDR_MEGA
                    and to_addr == self.ADDR_PC
                    and FC1 == 0
                    and FC2 == self.FC_READ_STATE
                ):
                    angle = self._mapAngleFromRobot(self.robot, motorID, response[4])
                    velocity = self._mapAngVelFromMotor(response[5])
                    return [angle, velocity]
                else:
                    print(
                        f"Error: Invalid response from {from_addr} to {to_addr} with FC1={FC1} and FC2={FC2}"
                    )
                    return -1

        except Exception as e:
            print("Error:", e)
            return -1

        return -1

    def _mapAngVelToMotor(self, vel: float):
        """
        Description:
        This function maps the angular velocity in rad/s to the 10-bit motor speed.
        Only magnitude matters. The sign is skipped.
        The motor will reach to the next position with 'vel' rad/s angular speed.
        Angular velocity mapping to/from robot is independent of the robot type and motor ID.

        Input:
        vel: The angular velocity in rad/s.
        The sign doesn't matter as its magnitude is used.

        Output:
        The 10-bit motor speed value.
        """

        # See https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#moving-speed
        # Input in rad/s with sign.

        scl = (1023 / 114) * (60 / (2 * math.pi))

        # Only magnitude matters. The sign is skipped.
        spd = abs(vel)

        # Zero speed is not allowed. Zero speed means full 1023 speed. So scale it a bit and add a constant.
        if spd < 0.1:
            spd = spd * 1.5 + 0.1

        # Convert to 10-bit value.
        op = int(spd * scl)

        # Clip the value to the range [0, 1023].
        op = self._clip(op, 0, 1023)

        return op

    def setJointState(self, motorID: int, state: list):
        """
        Description:
        This function sets the motor joint angle and joint velocity.
        The motor torque is enabled automatically.

        Input:
        motorID: The ID of the motor
        state: [joint angle (rad), joint velocity (rad/s)]

        Output:
        success/failure

        """

        # If the list is longer than 2, ignore the rest of the values
        if len(state) > 2:
            print(
                "Warning: The state list is longer than 2. Ignoring the rest of the values"
            )

        # Packet to send
        from_addr = self.ADDR_PC
        to_addr = self.ADDR_MEGA
        FC1 = self.FC_WRITE_STATE
        FC2 = 0
        # Packet
        my_array = list()
        my_array.append(from_addr)
        my_array.append(to_addr)
        my_array.append(FC1)
        my_array.append(FC2)
        my_array.append(motorID)
        motorAngle = self._mapAngleToRobot(self.rob, motorID, state[0])
        my_array.append(motorAngle)
        motorSpeed = self._mapAngVelToMotor(state[1])
        my_array.append(motorSpeed)

        try:
            # Load the Tx buffer with the packet
            send_size = self.link.tx_obj(my_array)
            # Send the packet
            self.link.send(send_size)
            # Wait for the packet to be sent
            sleep(0.001)
            # Record the time the packet was sent
            send_time = time.time()

            # Wait for a response
            data_recieved = True
            while not self.link.available():
                if self.link.status < 0:
                    if self.link.status == txfer.CRC_ERROR:
                        print("ERROR: CRC_ERROR")
                    elif self.link.status == txfer.PAYLOAD_ERROR:
                        print("ERROR: PAYLOAD_ERROR")
                    elif self.link.status == txfer.STOP_BYTE_ERROR:
                        print("ERROR: STOP_BYTE_ERROR")
                    else:
                        print("ERROR: {}".format(self.link.status))
                    return False
                sleep(0.001)
                # Check for time out
                if (time.time() - send_time) > self.RS485_TO:
                    print("Err: Time out")
                    data_recieved = False
                    return -1

            if data_recieved:
                # print(f"ByteRead: {self.link.bytesRead}")
                # Process the response if needed
                response = self.link.rx_obj(
                    obj_type=list, obj_byte_size=self.link.bytesRead, list_format="i"
                )
                from_addr = response[0]
                to_addr = response[1]
                FC1 = response[2]
                FC2 = response[3]

                if (
                    from_addr == self.ADDR_MEGA
                    and to_addr == self.ADDR_PC
                    and FC1 == 0
                    and FC2 == self.FC_WRITE_STATE
                ):
                    # Read the acknowledgement
                    ack = response[4]
                    return ack
                else:
                    print(
                        f"Error: Invalid response from {from_addr} to {to_addr} with FC1={FC1} and FC2={FC2}"
                    )
                    return -1

        except Exception as e:
            print("Error:", e)
            return -1

        return -1

    def getRobotState(self):
        """
        Description: This function reads the robot joint angles and joint velocities.

        Input: -NA-

        Output:
        For 2R robot: JA1, JV1 corresponds to the base motor.
        For 5bar robot: JA1, JV1 corresponds to the left motor.
        [[JA1, JV1],[JA2, JV2]] units: rad for angle, rad/s for velocity
        -1 if failed to get the state

        """

        # Packet to send
        from_addr = self.ADDR_PC
        to_addr = self.ADDR_MEGA
        FC1 = self.FC_READ_ROBOT_STATE
        FC2 = 0
        # Packet
        my_array = list()
        my_array.append(from_addr)
        my_array.append(to_addr)
        my_array.append(FC1)
        my_array.append(FC2)
        my_array.append(1)  # motor ID 1
        my_array.append(2)  # motor ID 2

        try:
            # Load the Tx buffer with the packet
            send_size = self.link.tx_obj(my_array)
            # Send the packet
            self.link.send(send_size)
            # Wait for the packet to be sent
            sleep(0.001)
            # Record the time the packet was sent
            send_time = time.time()

            # Wait for a response
            data_recieved = True
            while not self.link.available():
                if self.link.status < 0:
                    if self.link.status == txfer.CRC_ERROR:
                        print("ERROR: CRC_ERROR")
                    elif self.link.status == txfer.PAYLOAD_ERROR:
                        print("ERROR: PAYLOAD_ERROR")
                    elif self.link.status == txfer.STOP_BYTE_ERROR:
                        print("ERROR: STOP_BYTE_ERROR")
                    else:
                        print("ERROR: {}".format(self.link.status))
                    return False
                sleep(0.001)
                # Check for time out
                if (time.time() - send_time) > self.RS485_TO:
                    print("Err: Time out")
                    data_recieved = False
                    return -1

            if data_recieved:
                # print(f"ByteRead: {self.link.bytesRead}")
                # Process the response if needed
                response = self.link.rx_obj(
                    obj_type=list, obj_byte_size=self.link.bytesRead, list_format="i"
                )
                from_addr = response[0]
                to_addr = response[1]
                FC1 = response[2]
                FC2 = response[3]
                if (
                    from_addr == self.ADDR_MEGA
                    and to_addr == self.ADDR_PC
                    and FC1 == 0
                    and FC2 == self.FC_READ_ROBOT_STATE
                ):
                    angle1 = self._mapAngleFromRobot(self.rob, 1, response[4])
                    velocity1 = self._mapAngVelFromMotor(response[5])
                    angle2 = self._mapAngleFromRobot(self.rob, 2, response[6])
                    velocity2 = self._mapAngVelFromMotor(response[7])
                    return [[angle1, velocity1], [angle2, velocity2]]
                else:
                    print(
                        f"Error: Invalid response from {from_addr} to {to_addr} with FC1={FC1} and FC2={FC2}"
                    )
                    return -1

        except Exception as e:
            print("Error:", e)
            return -1

        return -1

    def setRobotState(self, state: list):
        """
        Description:
        This function sets the robot joint angles and joint velocities synchronously.
        The motor torques are enabled automatically.

        Input:
        state: [[JA1, JV1],[JA2, JV2]] units: rad for angle, rad/s for velocity

        Output:
        success/failure

        """

        # If the list is longer than 2, ignore the rest of the values
        if len(state) > 2:
            print(
                "Warning: The state list is longer than 2. Ignoring the rest of the values"
            )

        # Packet to send
        from_addr = self.ADDR_PC
        to_addr = self.ADDR_MEGA
        FC1 = self.FC_WRITE_ROBOT_STATE
        FC2 = 0
        # Packet
        my_array = list()
        my_array.append(from_addr)
        my_array.append(to_addr)
        my_array.append(FC1)
        my_array.append(FC2)
        my_array.append(1)  # motor ID 1
        my_array.append(2)  # motor ID 2
        motorAngle1 = self._mapAngleToRobot(self.rob, 1, state[0][0])
        my_array.append(motorAngle1)
        motorSpeed1 = self._mapAngVelToMotor(state[0][1])
        my_array.append(motorSpeed1)
        motorAngle2 = self._mapAngleToRobot(self.rob, 2, state[1][0])
        my_array.append(motorAngle2)
        motorSpeed2 = self._mapAngVelToMotor(state[1][1])
        my_array.append(motorSpeed2)

        try:
            # Load the Tx buffer with the packet
            send_size = self.link.tx_obj(my_array)
            # Send the packet
            self.link.send(send_size)
            # Wait for the packet to be sent
            sleep(0.001)
            # Record the time the packet was sent
            send_time = time.time()

            # Wait for a response
            data_recieved = True
            while not self.link.available():
                if self.link.status < 0:
                    if self.link.status == txfer.CRC_ERROR:
                        print("ERROR: CRC_ERROR")
                    elif self.link.status == txfer.PAYLOAD_ERROR:
                        print("ERROR: PAYLOAD_ERROR")
                    elif self.link.status == txfer.STOP_BYTE_ERROR:
                        print("ERROR: STOP_BYTE_ERROR")
                    else:
                        print("ERROR: {}".format(self.link.status))
                    return False
                sleep(0.001)
                # Check for time out
                if (time.time() - send_time) > self.RS485_TO:
                    print("Err: Time out")
                    data_recieved = False
                    return -1

            if data_recieved:
                # print(f"ByteRead: {self.link.bytesRead}")
                # Process the response if needed
                response = self.link.rx_obj(
                    obj_type=list, obj_byte_size=self.link.bytesRead, list_format="i"
                )
                from_addr = response[0]
                to_addr = response[1]
                FC1 = response[2]
                FC2 = response[3]

                if (
                    from_addr == self.ADDR_MEGA
                    and to_addr == self.ADDR_PC
                    and FC1 == 0
                    and FC2 == self.FC_WRITE_ROBOT_STATE
                ):
                    # Read the acknowledgement
                    ack = response[4]
                    return ack
                else:
                    print(
                        f"Error: Invalid response from {from_addr} to {to_addr} with FC1={FC1} and FC2={FC2}"
                    )
                    return -1

        except Exception as e:
            print("Error:", e)
            return -1

        return -1

    def goHome(self):
        """
        Description:
        This function moves the robot to the home position.

        Input: -NA-
        
        Output: success/failure
        
        """
        if self.rob == "2R":
            home_state = [[0, 0.5], [0, 0.5]]
        elif self.rob == "5bar":
            home_state = [[3 * math.pi / 4, 0.5], [math.pi / 4, 0.5]]
        
        self.penUp()
        sleep(0.5)
        ack = self.setRobotState(home_state)
        sleep(0.5)
        return ack

    def setGetRobotState(self, state: list):
        """
        Description:
        This function sets the robot joint angles and joint velocities synchronously.
        And then reads the instantaneous robot joint angles and joint velocities.

        Input:
        state: [[JA1, JV1],[JA2, JV2]] units: rad for angle, rad/s for velocity
        For 2R robot: JA1, JV1 corresponds to the base motor.
        For 5bar robot: JA1, JV1 corresponds to the left motor.

        Output:
        For 2R robot: JA1, JV1 corresponds to the base motor.
        For 5bar robot: JA1, JV1 corresponds to the left motor.
        [[JA1, JV1],[JA2, JV2]] units: rad for angle, rad/s for velocity
        -1 if failed to get the state

        """

        # Packet to send
        from_addr = self.ADDR_PC
        to_addr = self.ADDR_MEGA
        FC1 = self.FC_RW_ROBOT_STATE
        FC2 = 0
        # Packet
        my_array = list()
        my_array.append(from_addr)
        my_array.append(to_addr)
        my_array.append(FC1)
        my_array.append(FC2)
        my_array.append(1)  # motor ID 1
        my_array.append(2)  # motor ID 2
        motorAngle1 = self._mapAngleToRobot(self.rob, 1, state[0][0])
        my_array.append(motorAngle1)
        motorSpeed1 = self._mapAngVelToMotor(state[0][1])
        my_array.append(motorSpeed1)
        motorAngle2 = self._mapAngleToRobot(self.rob, 2, state[1][0])
        my_array.append(motorAngle2)
        motorSpeed2 = self._mapAngVelToMotor(state[1][1])
        my_array.append(motorSpeed2)

        try:
            # Load the Tx buffer with the packet
            send_size = self.link.tx_obj(my_array)
            # Send the packet
            self.link.send(send_size)
            # Wait for the packet to be sent
            sleep(0.001)
            # Record the time the packet was sent
            send_time = time.time()

            # Wait for a response
            data_recieved = True
            while not self.link.available():
                if self.link.status < 0:
                    if self.link.status == txfer.CRC_ERROR:
                        print("ERROR: CRC_ERROR")
                    elif self.link.status == txfer.PAYLOAD_ERROR:
                        print("ERROR: PAYLOAD_ERROR")
                    elif self.link.status == txfer.STOP_BYTE_ERROR:
                        print("ERROR: STOP_BYTE_ERROR")
                    else:
                        print("ERROR: {}".format(self.link.status))
                    return False
                sleep(0.001)
                # Check for time out
                if (time.time() - send_time) > self.RS485_TO:
                    print("Err: Time out")
                    data_recieved = False
                    return -1

            if data_recieved:
                # print(f"ByteRead: {self.link.bytesRead}")
                # Process the response if needed
                response = self.link.rx_obj(
                    obj_type=list, obj_byte_size=self.link.bytesRead, list_format="i"
                )
                from_addr = response[0]
                to_addr = response[1]
                FC1 = response[2]
                FC2 = response[3]

                if (
                    from_addr == self.ADDR_MEGA
                    and to_addr == self.ADDR_PC
                    and FC1 == 0
                    and FC2 == self.FC_RW_ROBOT_STATE
                ):
                    angle1 = self._mapAngleFromRobot(self.rob, 1, response[4])
                    velocity1 = self._mapAngVelFromMotor(response[5])
                    angle2 = self._mapAngleFromRobot(self.rob, 2, response[6])
                    velocity2 = self._mapAngVelFromMotor(response[7])
                    return [[angle1, velocity1], [angle2, velocity2]]
                else:
                    print(
                        f"Error: Invalid response from {from_addr} to {to_addr} with FC1={FC1} and FC2={FC2}"
                    )
                    return -1

        except Exception as e:
            print("Error:", e)
            return -1

        return -1
