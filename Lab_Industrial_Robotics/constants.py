import techman as tm
import dynamixel_sdk as dxl

# In this file the main constants for the motion planning and servomotor are defined

# Timeouts in the code to perform the vision task and gripping
vision_timeout = 0.5  # seconds
gripping_timeout = 2  # seconds

#%% SERVOMOTOR SETUP AND GRIPPER FUNCTION--------------------------------------------------
# Main parameters for servomotor connection
DEVICENAME = 'COM7' # Port number, to be modified depending on the computer
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0
DXL_ID = 1

# Memory adresses
ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POSITION = 36
ADDR_MOVING_SPEED = 32

# Conversion from ° to servomotor position
DEG_TO_POS = 1023 / 300  

# Connection to the robot through its IP adress
TM12 = tm.TM_Robot("192.168.1.1")

# Servomotor communication setup
port_handler = dxl.PortHandler(DEVICENAME)
packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)

port_handler.openPort()
port_handler.setBaudRate(BAUDRATE)


# Set initial position to 0
current_position = 0
pos = int(current_position*DEG_TO_POS)
packet_handler.write2ByteTxRx(port_handler, DXL_ID, ADDR_GOAL_POSITION, pos)

