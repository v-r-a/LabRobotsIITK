// Code to be burnt on Arduino Mega of 5bar parallel robot with
// Dynamixel-Arduino-IITK shield.
// Two dynamixel motors with ID 1 and ID 2 are expected
// To be used with PySerialTransfer on PC
// Communication packet:
// From (1B) | To (1B) | FunctionCode1 (1B) | FC2 (1B) | DATA

// Libraries
#include <DynamixelShield.h>
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#define DEBUG_SERIAL Serial1
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
#define DEBUG_SERIAL SerialUSB
#else
#define DEBUG_SERIAL Serial
#endif
#include "SerialTransfer.h"
#include <Servo.h>
// Function declarations


/***********************************************************************************************/
// Macros
#define DXL_LOBYTE(w) ((uint8_t)(((uint16_t)(w)) & 0xff))
#define DXL_HIBYTE(w) ((uint8_t)((((uint16_t)(w)) >> 8) & 0xff))
/***********************************************************************************************/
// Constants
#define BAUDRATE_DXL 1000000
#define BAUDRATE 1000000
#define RS485_RE_DE_PIN 3  // Based on indigenous Dynamixel_Arduino_Shield.
#define PEN_SERVO_PIN 9
const unsigned long RS485_RDT = 1000;
const unsigned long DATA_OUT_SLEEP = 600;

const long ADDR_PC = 100;
const long ADDR_MEGA = 200;

const long FC_PING = 1;
const long FC_TORQUE_ON = 2;
const long FC_TORQUE_OFF = 3;
const long FC_TORQUE_STATUS = 4;
const long FC_READ_ANGLE = 5;
const long FC_WRITE_ANGLE = 6;
const long FC_PEN_SERVO = 7;
const long FC_READ_STATE = 8;
const long FC_WRITE_STATE = 9;
const long FC_READ_ROBOT_STATE = 10;
const long FC_WRITE_ROBOT_STATE = 11;
const long FC_RW_ROBOT_STATE = 12;
/***********************************************************************************************/

// Control table address for AX-12A
const uint8_t ADDR_AX_TORQUE_ENABLE = 24;
const uint8_t ADDR_AX_GOAL_POSITION = 30;
const uint8_t ADDR_AX_MOVING_SPEED = 32;
const uint8_t ADDR_AX_PRESENT_POSITION = 36;
const uint8_t ADDR_AX_CW_COMP_MARGIN = 26;
const uint8_t ADDR_AX_CCW_COMP_MARGIN = 27;
const uint8_t ADDR_AX_CW_COMP_SLOPE = 28;
const uint8_t ADDR_AX_CCW_COMP_SLOPE = 29;
const uint8_t ADDR_AX_PRESENT_LOAD = 40;
const uint8_t ADDR_AX_LOCK_EEPROM = 47;
const uint8_t ADDR_AX_RDT = 5;

// Data Length in Bytes
const uint8_t LEN_AX_GOAL_POSITION = 2;
const uint8_t LEN_AX_MOVING_SPEED = 2;
const uint8_t LEN_AX_PRESENT_POSITION = 2;
const uint8_t LEN_AX_CW_COMP_MARGIN = 1;
const uint8_t LEN_AX_CCW_COMP_MARGIN = 1;
const uint8_t LEN_AX_CW_COMP_SLOPE = 1;
const uint8_t LEN_AX_CCW_COMP_SLOPE = 1;
const uint8_t LEN_AX_PRESENT_LOAD = 2;
const uint8_t LEN_AX_LOCK_EEPROM = 1;
const uint8_t LEN_AX_RDT = 1;

// Other const
const uint8_t TORQUE_ENABLE = 1;
const uint8_t TORQUE_DISABLE = 0;
const uint8_t LOCK_EEPROM = 1;

const uint8_t NMOTOR = 2;
const uint8_t MOTOR_ID_LIST[NMOTOR] = { 1, 2 };

// Homing position and return to home velocity
uint16_t home_pos_list[NMOTOR] = { 512 - 153, 512 + 153 };
uint16_t home_vel_list[NMOTOR] = { 50, 50 };

/***********************************************************************************************/
// Protocol version
const float PROTOCOL_VERSION = 1.0;
// Starting address of the Data to write; Goal Position = 30
const uint16_t SW_START_ADDR = ADDR_AX_GOAL_POSITION;
// Length of the Data to write: 4 bytes: Goal Position + Goal Velocity
const uint16_t SW_ADDR_LEN = 4;
// Four columns and NMOTOR rows
uint8_t myswdata[NMOTOR][SW_ADDR_LEN];
/***********************************************************************************************/
// Global variables declarations
SerialTransfer myTransfer;                               // Communication over RS 485
DynamixelShield dxl;                                     // Communication with motors
DYNAMIXEL::InfoSyncWriteInst_t allmotorSW;               // Dynamixel syncwrite
DYNAMIXEL::XELInfoSyncWrite_t all_info_xels_sw[NMOTOR];  // Dynamixel syncwrite
bool write_feedback;                                     // debugging
Servo myservo;                                           // Servo object to move the pen

void setup() {
  // RS 485 on UART #3
  Serial3.begin(BAUDRATE);
  delay(10);
  myTransfer.begin(Serial3);
  delay(10);
  pinMode(RS485_RE_DE_PIN, OUTPUT);
  digitalWrite(RS485_RE_DE_PIN, LOW);  // Listening to the PC
  // RS 485 receipt LED
  pinMode(LED_BUILTIN, OUTPUT);

  // DXL comms
  dxl.setPortProtocolVersion(1.0);
  dxl.begin(BAUDRATE_DXL);

  // DEBUG_SERIAL
  DEBUG_SERIAL.begin(115200);

  // Torque ON and other One time settings
  uint8_t motor_id;
  // Write only once control table items
  for (int i = 0; i < NMOTOR; i++) {
    motor_id = MOTOR_ID_LIST[i];
    DEBUG_SERIAL.print("MOTOR ID: ");
    DEBUG_SERIAL.println(motor_id);
    // Redefine the packet return delay time (1 = 2us)
    write_feedback = dxl.writeControlTableItem(ControlTableItem::RETURN_DELAY_TIME, motor_id, 1);
    DEBUG_SERIAL.print("Return delay time set (1 for success): ");
    DEBUG_SERIAL.println(write_feedback);
    delay(10);
    // Lock the EEPROM
    // write_feedback = dxl.writeControlTableItem(LOCK, motor_id, LOCK_EEPROM);
    // DEBUG_SERIAL.print("EEPROM locked (1 for success): ");
    // DEBUG_SERIAL.println(write_feedback);
    delay(10);

    // Set compliance margin and compliance slope
    write_feedback = dxl.writeControlTableItem(ControlTableItem::CW_COMPLIANCE_MARGIN, motor_id, 2);
    DEBUG_SERIAL.print("CW Compliance Margin set (1 for success): ");
    DEBUG_SERIAL.println(write_feedback);
    delay(10);

    write_feedback = dxl.writeControlTableItem(ControlTableItem::CCW_COMPLIANCE_MARGIN, motor_id, 2);
    DEBUG_SERIAL.print("CCW Compliance Margin set (1 for success): ");
    DEBUG_SERIAL.println(write_feedback);
    delay(10);

    write_feedback = dxl.writeControlTableItem(ControlTableItem::CW_COMPLIANCE_SLOPE, motor_id, 16);
    DEBUG_SERIAL.print("CW Compliance Slope set (1 for success): ");
    DEBUG_SERIAL.println(write_feedback);
    delay(10);

    write_feedback = dxl.writeControlTableItem(ControlTableItem::CCW_COMPLIANCE_SLOPE, motor_id, 16);
    DEBUG_SERIAL.print("CCW Compliance Slope set (1 for success): ");
    DEBUG_SERIAL.println(write_feedback);
    delay(10);

    // Enable Torques one by one
    write_feedback = dxl.torqueOn(motor_id);
    DEBUG_SERIAL.print("Torque ON (1 for success): ");
    DEBUG_SERIAL.println(write_feedback);
    delay(10);

    // Set the servo position to pen up position
    myservo.attach(PEN_SERVO_PIN);
    myservo.write(0);  // pen up
    // myservo.write(30);  // pen down
    delay(10);
  }
  /***********************************************************************************************/
  // Fill the members of structure to syncWrite using internal packet buffer
  // (This part of code is equivalent to 'dummy syncwrite to start off' in c++ Dynamixel SDK.)
  // All motors sync write
  allmotorSW.packet.p_buf = nullptr;
  allmotorSW.packet.is_completed = false;
  allmotorSW.addr = SW_START_ADDR;
  allmotorSW.addr_length = SW_ADDR_LEN;
  allmotorSW.p_xels = all_info_xels_sw;
  allmotorSW.xel_count = 0;

  // AddParam equivalent: Go to the home pose
  for (int i = 0; i < NMOTOR; i++) {
    myswdata[i][0] = DXL_LOBYTE(home_pos_list[i]);  // low byte
    myswdata[i][1] = DXL_HIBYTE(home_pos_list[i]);  // high byte
    myswdata[i][2] = DXL_LOBYTE(home_vel_list[i]);  // low byte
    myswdata[i][3] = DXL_HIBYTE(home_vel_list[i]);  // high byte
    all_info_xels_sw[i].id = MOTOR_ID_LIST[i];
    all_info_xels_sw[i].p_data = myswdata[i];
    allmotorSW.xel_count++;
  }
  allmotorSW.is_info_changed = true;
  dxl.syncWrite(&allmotorSW);
  // Allow some time for going to home position
  delay(3000);
}

void loop() {
  // Variables related to incomming packet (4-byte integers from PC)
  long packetIn[20];
  long addr_from, addr_to, fc1, fc2, len_packet;
  long *dataIn;

  // Variables related to outgoing packet
  int ptr = 0;
  long dataOut[32];
  uint8_t packetOut[64];
  // Reset LED state
  digitalWrite(LED_BUILTIN, LOW);

  // Read incomming packet from PC, if any:
  if (myTransfer.available()) {
    // Recieved something! Light up the orange built-in LED
    digitalWrite(LED_BUILTIN, HIGH);

    // Read the packet
    len_packet = myTransfer.bytesRead;          // length of the packet
    myTransfer.rxObj(packetIn, 0, len_packet);  // read it
    addr_from = packetIn[0];                    // packet from whom?
    addr_to = packetIn[1];                      // addressed to whom?
    fc1 = packetIn[2];                          // what to do?
    fc2 = packetIn[3];                          // what to do more?
    dataIn = packetIn + 4;                      // pointer to the data in the packet

    // DEBUG print
    DEBUG_SERIAL.println(addr_from);
    DEBUG_SERIAL.println(addr_to);
    DEBUG_SERIAL.println(fc1);
    DEBUG_SERIAL.println(fc2);
    DEBUG_SERIAL.println(dataIn[0]);
    // DEBUG_SERIAL.print("FC1 match result: ");
    // DEBUG_SERIAL.println(fc1 == FC_READ_STATE);

    // If the data has been sent by the PC and addressed to me, then:
    if (addr_from == ADDR_PC && addr_to == ADDR_MEGA) {
      DEBUG_SERIAL.println("Hello FROM-TO match");
      // Reply timer start
      unsigned long reply_timer_start = micros();
      // Do action based on the FC1
      if (fc1 == FC_PING) {
        // Ping the motor and note the reply
        bool ping_op = dxl.ping((uint8_t)dataIn[0]);
        DEBUG_SERIAL.println("In PING:");
        DEBUG_SERIAL.println(ping_op);
        // Start building the return packet
        dataOut[0] = ADDR_MEGA;      // From
        dataOut[1] = ADDR_PC;        // To
        dataOut[2] = 0;              // FC1
        dataOut[3] = FC_PING;        // FC2
        dataOut[4] = (long)ping_op;  // Data
        ptr = 5 * sizeof(long);
        memcpy(packetOut, dataOut, ptr);
      } else if (fc1 == FC_TORQUE_ON) {
        // Send command to the motor and note the reply
        bool tqOn_op = dxl.torqueOn((uint8_t)dataIn[0]);
        DEBUG_SERIAL.println("In TorqueOn:");
        DEBUG_SERIAL.println(tqOn_op);
        // Start building the return packet
        dataOut[0] = ADDR_MEGA;      // From
        dataOut[1] = ADDR_PC;        // To
        dataOut[2] = 0;              // FC1
        dataOut[3] = FC_TORQUE_ON;   // FC2
        dataOut[4] = (long)tqOn_op;  // Data
        ptr = 5 * sizeof(long);
        memcpy(packetOut, dataOut, ptr);
      } else if (fc1 == FC_TORQUE_OFF) {
        // Send command to the motor and note the reply
        bool tqOff_op = dxl.torqueOff((uint8_t)dataIn[0]);
        DEBUG_SERIAL.println("In TorqueOff:");
        DEBUG_SERIAL.println(tqOff_op);
        // Start building the return packet
        dataOut[0] = ADDR_MEGA;       // From
        dataOut[1] = ADDR_PC;         // To
        dataOut[2] = 0;               // FC1
        dataOut[3] = FC_TORQUE_OFF;   // FC2
        dataOut[4] = (long)tqOff_op;  // Data
        ptr = 5 * sizeof(long);
        memcpy(packetOut, dataOut, ptr);
      } else if (fc1 == FC_TORQUE_STATUS) {
        DEBUG_SERIAL.print("In TorqueStatus:");
        // Start building the return packet
        dataOut[0] = ADDR_MEGA;         // From
        dataOut[1] = ADDR_PC;           // To
        dataOut[2] = 0;                 // FC1
        dataOut[3] = FC_TORQUE_STATUS;  // FC2
        // Send command to the motor and note the reply
        uint8_t torqStatusBuff[2];
        int torqStatus = dxl.read((uint8_t)dataIn[0], ADDR_AX_TORQUE_ENABLE, 1, torqStatusBuff, 2);
        dataOut[4] = (long)torqStatusBuff[0];
        DEBUG_SERIAL.println(dataOut[4]);
        // Failure to read the torque
        if (!torqStatus) {
          dataOut[4] = -1;
        };
        ptr = 5 * sizeof(long);
        memcpy(packetOut, dataOut, ptr);
        DEBUG_SERIAL.println(dataOut[4]);
      } else if (fc1 == FC_READ_ANGLE) {
        DEBUG_SERIAL.print("In ReadAngle: ");
        // Start building the return packet
        dataOut[0] = ADDR_MEGA;      // From
        dataOut[1] = ADDR_PC;        // To
        dataOut[2] = 0;              // FC1
        dataOut[3] = FC_READ_ANGLE;  // FC2
        // Send command to the motor and note the reply
        uint8_t readAngleBuff[2];
        int readAngleStatus = dxl.read((uint8_t)dataIn[0], ADDR_AX_PRESENT_POSITION, 2, readAngleBuff, 2);
        dataOut[4] = readAngleBuff[0] | (readAngleBuff[1] << 8);
        if (!readAngleStatus) {
          dataOut[4] = -1;
        }
        ptr = 5 * sizeof(long);
        memcpy(packetOut, dataOut, ptr);
        DEBUG_SERIAL.println(dataOut[4]);
      } else if (fc1 == FC_READ_STATE) {
        DEBUG_SERIAL.print("In ReadState: ");
        // Start building the return packet
        dataOut[0] = ADDR_MEGA;      // From
        dataOut[1] = ADDR_PC;        // To
        dataOut[2] = 0;              // FC1
        dataOut[3] = FC_READ_STATE;  // FC2
        // Send command to the motor and note the reply
        uint8_t readStateBuff[4];
        int readStateStatus = dxl.read((uint8_t)dataIn[0], ADDR_AX_PRESENT_POSITION, 4, readStateBuff, 4);
        dataOut[4] = readStateBuff[0] | (readStateBuff[1] << 8);  // POS
        dataOut[5] = readStateBuff[2] | (readStateBuff[3] << 8);  // VEL
        if (!readStateStatus) {
          dataOut[4] = -1;
        }
        ptr = 6 * sizeof(long);
        memcpy(packetOut, dataOut, ptr);
        DEBUG_SERIAL.println(String(dataOut[4]) + "," + String(dataOut[5]));
      } else if (fc1 == FC_READ_ROBOT_STATE) {
        DEBUG_SERIAL.print("In ReadRobotState: ");
        // Start building the return packet
        dataOut[0] = ADDR_MEGA;            // From
        dataOut[1] = ADDR_PC;              // To
        dataOut[2] = 0;                    // FC1
        dataOut[3] = FC_READ_ROBOT_STATE;  // FC2
        // Send command to the motor and note the reply
        uint8_t readStateBuff[8];
        int readStateStatus1 = dxl.read(1, ADDR_AX_PRESENT_POSITION, 4, readStateBuff, 4);
        int readStateStatus2 = dxl.read(2, ADDR_AX_PRESENT_POSITION, 4, readStateBuff + 4, 4);
        dataOut[4] = readStateBuff[0] | (readStateBuff[1] << 8);  // POS1
        dataOut[5] = readStateBuff[2] | (readStateBuff[3] << 8);  // VEL1
        dataOut[6] = readStateBuff[4] | (readStateBuff[5] << 8);  // POS2
        dataOut[7] = readStateBuff[6] | (readStateBuff[7] << 8);  // VEL2
        bool readStateStatus = readStateStatus1 && readStateStatus2;
        if (!readStateStatus) {
          dataOut[4] = -1;
        }
        ptr = 8 * sizeof(long);
        memcpy(packetOut, dataOut, ptr);
        DEBUG_SERIAL.println(String(dataOut[4]) + "," + String(dataOut[5]) + "," + String(dataOut[6]) + "," + String(dataOut[7]));
      } else if (fc1 == FC_WRITE_ANGLE) {
        DEBUG_SERIAL.print("In WriteAngle: ");
        // Start building the return packet
        dataOut[0] = ADDR_MEGA;       // From
        dataOut[1] = ADDR_PC;         // To
        dataOut[2] = 0;               // FC1
        dataOut[3] = FC_WRITE_ANGLE;  // FC2
        // Send command to the motor and note the reply
        bool writeAngleStatus = dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, dataIn[0], dataIn[1]);
        dataOut[4] = (long)writeAngleStatus;
        ptr = 5 * sizeof(long);
        memcpy(packetOut, dataOut, ptr);
        DEBUG_SERIAL.println(dataOut[4]);
      } else if (fc1 == FC_WRITE_STATE) {
        DEBUG_SERIAL.print("In WriteState: ");
        // Start building the return packet
        dataOut[0] = ADDR_MEGA;       // From
        dataOut[1] = ADDR_PC;         // To
        dataOut[2] = 0;               // FC1
        dataOut[3] = FC_WRITE_STATE;  // FC2
        // Send command to the motor and note the reply
        bool writeStateStatusPos = dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, dataIn[0], dataIn[1]);
        bool writeStateStatusVel = dxl.writeControlTableItem(ControlTableItem::MOVING_SPEED, dataIn[0], dataIn[2]);
        dataOut[4] = (long)(writeStateStatusPos && writeStateStatusVel);
        ptr = 5 * sizeof(long);
        memcpy(packetOut, dataOut, ptr);
        DEBUG_SERIAL.println(dataOut[4]);
      } else if (fc1 == FC_WRITE_ROBOT_STATE) {
        DEBUG_SERIAL.print("In WriteRobotState: ");
        // Start building the return packet
        dataOut[0] = ADDR_MEGA;             // From
        dataOut[1] = ADDR_PC;               // To
        dataOut[2] = 0;                     // FC1
        dataOut[3] = FC_WRITE_ROBOT_STATE;  // FC2
        // Send command to the motors synchronously and note the reply
        // uint8_t motor_id_list[2] = {dataIn[0],dataIn[1]}; // not using this
        allmotorSW.xel_count = 0;
        for (int i = 0; i < NMOTOR; i++) {
          myswdata[i][0] = lowByte(dataIn[2 + 2 * i]);   // pos low byte
          myswdata[i][1] = highByte(dataIn[2 + 2 * i]);  // pos high byte
          myswdata[i][2] = lowByte(dataIn[3 + 2 * i]);   // spd low byte
          myswdata[i][3] = highByte(dataIn[3 + 2 * i]);  // spd high byte
          all_info_xels_sw[i].id = MOTOR_ID_LIST[i];     // data corresponding to which motor?
          all_info_xels_sw[i].p_data = myswdata[i];      // update the data structure
          allmotorSW.xel_count++;                        // udpate the motor count
        }
        allmotorSW.is_info_changed = true;      // this is new data and needs to be written
        bool ack = dxl.syncWrite(&allmotorSW);  // write to motors
        // dxl.getLastLibErrCode()
        dataOut[4] = (long)ack;
        ptr = 5 * sizeof(long);
        memcpy(packetOut, dataOut, ptr);
        DEBUG_SERIAL.println(dataOut[4]);
      } else if (fc1 == FC_RW_ROBOT_STATE) {
        DEBUG_SERIAL.print("In RW RobotState: ");
        // Start building the return packet
        dataOut[0] = ADDR_MEGA;          // From
        dataOut[1] = ADDR_PC;            // To
        dataOut[2] = 0;                  // FC1
        dataOut[3] = FC_RW_ROBOT_STATE;  // FC2
        // Send command to the motors synchronously and note the reply
        // uint8_t motor_id_list[2] = {dataIn[0],dataIn[1]}; // not using this
        allmotorSW.xel_count = 0;
        for (int i = 0; i < NMOTOR; i++) {
          myswdata[i][0] = lowByte(dataIn[2 + 2 * i]);   // pos low byte
          myswdata[i][1] = highByte(dataIn[2 + 2 * i]);  // pos high byte
          myswdata[i][2] = lowByte(dataIn[3 + 2 * i]);   // spd low byte
          myswdata[i][3] = highByte(dataIn[3 + 2 * i]);  // spd high byte
          all_info_xels_sw[i].id = MOTOR_ID_LIST[i];     // data corresponding to which motor?
          all_info_xels_sw[i].p_data = myswdata[i];      // update the data structure
          allmotorSW.xel_count++;                        // udpate the motor count
        }
        allmotorSW.is_info_changed = true;      // this is new data and needs to be written
        bool ack = dxl.syncWrite(&allmotorSW);  // write to motors
        // dxl.getLastLibErrCode()
        // Send command to the motor and note the reply
        uint8_t readStateBuff[8];
        int readStateStatus1 = dxl.read(1, ADDR_AX_PRESENT_POSITION, 4, readStateBuff, 4);
        int readStateStatus2 = dxl.read(2, ADDR_AX_PRESENT_POSITION, 4, readStateBuff + 4, 4);
        dataOut[4] = readStateBuff[0] | (readStateBuff[1] << 8);  // POS1
        dataOut[5] = readStateBuff[2] | (readStateBuff[3] << 8);  // VEL1
        dataOut[6] = readStateBuff[4] | (readStateBuff[5] << 8);  // POS2
        dataOut[7] = readStateBuff[6] | (readStateBuff[7] << 8);  // VEL2
        bool readStateStatus = readStateStatus1 && readStateStatus2;
        if (!readStateStatus) {
          dataOut[4] = -1;
        }
        ptr = 8 * sizeof(long);
        memcpy(packetOut, dataOut, ptr);
        DEBUG_SERIAL.println(String(dataOut[4]) + "," + String(dataOut[5]) + String(dataOut[6]) + "," + String(dataOut[7]));
      } else if (fc1 == FC_PEN_SERVO) {
        DEBUG_SERIAL.print("In Pen Servo: ");
        // Start building the return packet
        dataOut[0] = ADDR_MEGA;     // From
        dataOut[1] = ADDR_PC;       // To
        dataOut[2] = 0;             // FC1
        dataOut[3] = FC_PEN_SERVO;  // FC2
        // Send command to the servo
        myservo.write(dataIn[0]);  // 30 degrees for pen down
        dataOut[4] = 1;            // useless. no feedback available
        ptr = 5 * sizeof(long);
        memcpy(packetOut, dataOut, ptr);
        DEBUG_SERIAL.println(dataOut[4]);
      } else {
        // Build the packet
        DEBUG_SERIAL.println("In default");
        dataOut[0] = ADDR_MEGA;  // From
        dataOut[1] = ADDR_PC;    // To
        dataOut[2] = 0;          // Sending nothing
        dataOut[3] = 0;          // useless
        dataOut[4] = 0;
        ptr = 5 * sizeof(long);
        memcpy(packetOut, dataOut, ptr);
      }

      // Load the packet into the transmit buffer
      myTransfer.txObj(packetOut, 0, ptr);

      // Wait for your time slot of reply
      while ((micros() - reply_timer_start) < RS485_RDT) {
        // Do nothing
      }

      // Send data
      DEBUG_SERIAL.print("Sending data now: ");
      DEBUG_SERIAL.println(ptr);
      digitalWrite(RS485_RE_DE_PIN, HIGH);
      myTransfer.sendData(ptr);
      delayMicroseconds(DATA_OUT_SLEEP);
      digitalWrite(RS485_RE_DE_PIN, LOW);
    }
  }
}
