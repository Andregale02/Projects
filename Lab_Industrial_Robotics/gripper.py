import time
from constants import *

# In this file the gripper operation function is defined

def gripper_action(action):

    # The function is structured depending on the action to be performed
    
    if action == "open": # Gripper opening
        target_position = int(0 * DEG_TO_POS)
        packet_handler.write2ByteTxRx(port_handler, DXL_ID, ADDR_GOAL_POSITION, target_position)
        print(f"Nuova posizione: {target_position/DEG_TO_POS}°")
        time.sleep(1)

    if action == "close": # Gripper closing 
        target_position = int(250 * DEG_TO_POS)
        packet_handler.write2ByteTxRx(port_handler, DXL_ID, ADDR_GOAL_POSITION, target_position)
        print(f"Nuova posizione: {target_position/DEG_TO_POS}°")
        time.sleep(1)

    if action == "visor": # Gripper in a particular position for visor closing
        target_position = int(190 * DEG_TO_POS)
        packet_handler.write2ByteTxRx(port_handler, DXL_ID, ADDR_GOAL_POSITION, target_position)
        print(f"Nuova posizione: {target_position/DEG_TO_POS}°")
        time.sleep(1)
