import time
from motion_parameters import *
from motion_steps import *
from gripper import gripper_action
from constants import *
import numpy as np

# In this file the actual movement is defined as the combination of the function included in the file motion_steps.py and gripper.py, 
# following the parameters contained in motion_parameters.py

# P.S. Only the first sequence is commented since all the others are the same except for parameter computaion (the sequence one motion contains sequence 2)
def sequence_1_M(ui):

    # Computation of motion parameters
    center_c, radius_c, deltar, max_angle_deltar, num_points_c , orientation_c, start_angle_c, end_angle_c = chinguard_motion_parameters_M()
    center_v, radius_v, deltar_v, max_angle_deltar_v, num_points_v , orientation_v, start_angle_v, end_angle_v = visor_motion_parameters_M()
    
    # VISOR OPENING------------------------------------------------------------------------

    # Gripper opening
    gripper_action("visor")
    
    # The initial point of the movement is defined 
    point_zero = [
        center_c[0] + (radius_c + 50) * math.cos(math.radians(start_angle_v)), 
        center_c[1],  
        center_c[2] + (radius_c + 50) * math.sin(math.radians(start_angle_v)),  
        orientation_v[0], orientation_v[1], orientation_v[2]  
    ]
    
    # Movement to the intial point
    TM12.ptp(point_zero,50)

    # Unlocking movement for the chinguard
    point_0 = visor_locking_up_M(center_v, radius_v, start_angle_v, orientation_v)
    
    # Motion upward of the chinguard
    point_1 = visor_motion_up_M(center_v, radius_v, max_angle_deltar_v, deltar_v, num_points_v, start_angle_v, end_angle_v, orientation_v)
    
    # Check if the end effector is in the right position before performing the vision task
    check_tcp_coord(point_1)    

    # Exit from listen node to perform vision job
    TM12.exit('1')
    time.sleep(vision_timeout)

    # Confirm if the movement has been performed and update the log on the UI
    confirm_movement(ui,"\nVisor motion up completed")
    
    #CHINGUARD OPENING----------------------------------------------------------------------
    
    # Definition of the starting point for the chinguard motion
    point_2 = [
        center_c[0] + (radius_c + deltar) * math.cos(math.radians(start_angle_c)), 
        center_c[1],  
        center_c[2] + (radius_c + deltar) * math.sin(math.radians(start_angle_c)),  
        orientation_c[0], orientation_c[1], orientation_c[2]  
    ]
    
    # Transfer to the previously defined point
    point_2 = transfer(point_1, point_2, center_c, radius_c)

    # Gripper opening
    gripper_action("open")

    # Unlocking of the chinguard
    point_2_5=chinguard_locking_M(center_c, radius_c, deltar, start_angle_c+1, orientation_c)

    # Confirm that the end effector is in the right place before closing the gripper
    check_tcp_coord(point_2_5)

    # Gripper closing
    gripper_action("close")

    # Wait for the gripper to close the helmet
    time.sleep(gripping_timeout)

    # Motion upward of the chinguard
    point_3 = chinguard_motion_M("up",center_c, radius_c, max_angle_deltar, deltar, num_points_c, start_angle_c, end_angle_c-1, orientation_c)
    
    # Confirm that the end effector is in the right place before opening the gripper
    check_tcp_coord(point_3)

    # Gripper oepning
    gripper_action("open")

    # Quit listen node to perform vision task
    TM12.exit('1')
    time.sleep(vision_timeout)

    # Confirm if the movement has been performed and update the log on the UI
    confirm_movement(ui,"\nChinguard motion up completed")

    #VISOR CLOSING------------------------------------------------------------------------
    
    # Move to visor up position
    point_4 = transfer(point_3, point_1, center_c, radius_c)

    # Position the gripper in the right position for visor motion
    gripper_action("visor")

    # Motion downward of the visor
    point_5 = visor_motion_down_M(center_v, radius_v, deltar, num_points_v, start_angle_v, end_angle_v, orientation_v)
    
    # Confirm that the end effector is in the right place before performing vision task
    check_tcp_coord(point_5)

    # Quit listen node to perform vision task
    TM12.exit('1')
    time.sleep(vision_timeout)

    # Confirm if the movement has been performed and update the log on the UI
    confirm_movement(ui,"\nVisor motion down completed")

    #CHINGUARD CLOSING--------------------------------------------------------------------
    
    # Move to chinguard up position
    point_6 = transfer(point_5, point_3, center_c, radius_c) 
    
    # Open the gripper during the motion
    gripper_action("open")

    # Unlock chinguard in the backward position
    point6_5=chinguard_locking_M(center_c, radius_c-10, deltar, end_angle_c-1, orientation_c)
    
    # Confirm that the end effector is in the right place before closing the gripper
    check_tcp_coord(point6_5)

    # Gripper closing
    gripper_action("close")
    time.sleep(gripping_timeout)

    # Motion downward of the chinguard
    point_7 = chinguard_motion_M("down",center_c, radius_c, max_angle_deltar, deltar, num_points_c, start_angle_c-1.5, end_angle_c-1, orientation_c)
    
    # Confirm that the end effector is in the right place before opening the visor and performing vision task
    check_tcp_coord(point_7)
    gripper_action("open")

    # Quit listen node to perform vision task
    TM12.exit('1')        
    time.sleep(vision_timeout)

    # Confirm the execution and update log
    confirm_movement(ui,"\nChinguard motion down completed")
    
    #VISOR CLOSING----------------------------------------------------------------------
    
    # Move to visor up position
    point_8 = transfer(point_2, point_1, center_c, radius_c)

    # Place gripper in visor position
    gripper_action("visor")

    # Downward motion of the visor
    point_9 = visor_motion_down_M(center_v, radius_v, deltar, num_points_v, start_angle_v, end_angle_v, orientation_v)

    # Confirm to be in the right position before performing the vision job
    check_tcp_coord(point_9)

    # Quit listen node to perform vision job
    TM12.exit('1')
    time.sleep(vision_timeout)
    
    # Confirm the execution and update log
    confirm_movement(ui,"\nVisor motion down completed")

def sequence_2_M(ui):

    # Computation of motion parameters
    center_c, radius_c, deltar, max_angle_deltar, num_points_c , orientation_c, start_angle_c, end_angle_c = chinguard_motion_parameters_M()
    center_v, radius_v, deltar_v, max_angle_deltar_v, num_points_v , orientation_v, start_angle_v, end_angle_v = visor_motion_parameters_M()
    
    #VISOR OPENING----------------------------------------------------------------------
    gripper_action("visor")
    point_zero = [
        center_c[0] + (radius_c + 50) * math.cos(math.radians(start_angle_v)), 
        center_c[1],  
        center_c[2] + (radius_c + 50) * math.sin(math.radians(start_angle_v)),  
        orientation_v[0], orientation_v[1], orientation_v[2]  
    ]
    TM12.ptp(point_zero,50)
    point_0 = visor_locking_up_M(center_v, radius_v, start_angle_v, orientation_v)
    point_1 = visor_motion_up_M(center_v, radius_v, max_angle_deltar_v, deltar_v, num_points_v, start_angle_v, end_angle_v, orientation_v)
    check_tcp_coord(point_1)    
    TM12.exit('1')

    time.sleep(vision_timeout)
    confirm_movement(ui,"\nVisor motion up completed")

    #CHINGUARD OPENING----------------------------------------------------------------------
    
    point_2 = [
        center_c[0] + (radius_c + deltar) * math.cos(math.radians(start_angle_c)), 
        center_c[1],  
        center_c[2] + (radius_c + deltar) * math.sin(math.radians(start_angle_c)),  
        orientation_c[0], orientation_c[1], orientation_c[2]  
    ]
    point_2 = transfer(point_1, point_2, center_c, radius_c)
    gripper_action("open")
    point_2_5=chinguard_locking_M(center_c, radius_c, deltar, start_angle_c+1, orientation_c)
    check_tcp_coord(point_2_5)
    gripper_action("close")
    time.sleep(gripping_timeout)
    point_3 = chinguard_motion_M("up",center_c, radius_c, max_angle_deltar, deltar, num_points_c, start_angle_c, end_angle_c-1, orientation_c)
    check_tcp_coord(point_3)
    gripper_action("open")
    TM12.exit('1')
    
    time.sleep(vision_timeout)
    confirm_movement(ui,"\nChinguard motion up completed")

    
    point6_5=chinguard_locking_M(center_c, radius_c-10, deltar, end_angle_c-1, orientation_c)
    check_tcp_coord(point6_5)
    gripper_action("close")
    time.sleep(gripping_timeout)
    point_7 = chinguard_motion_M("down",center_c, radius_c, max_angle_deltar, deltar, num_points_c, start_angle_c-1.5, end_angle_c-1, orientation_c)
    check_tcp_coord(point_7)
    gripper_action("open")
    TM12.exit('1')        

    time.sleep(vision_timeout)
    confirm_movement(ui,"\nChinguard motion down completed")

    #VISOR CLOSING----------------------------------------------------------------------
    
    point_8 = transfer(point_2, point_1, center_c, radius_c)
    gripper_action("visor")
    point_9 = visor_motion_down_M(center_v, radius_v, deltar, num_points_v, start_angle_v, end_angle_v, orientation_v)
    check_tcp_coord(point_9)
    TM12.exit('1')
    
    time.sleep(vision_timeout)
    confirm_movement(ui,"\nVisor motion down completed")

def sequence_1_XXXL(ui):

    #computation of motion parameters
    center_c, radius_c, deltar, max_angle_deltar, num_points_c , orientation_c, start_angle_c, end_angle_c = chinguard_motion_parameters_XXXL()
    center_v, radius_v, deltar_v, max_angle_deltar_v, num_points_v , orientation_v, start_angle_v, end_angle_v = visor_motion_parameters_XXXL()

    #VISOR OPENING-------------------------------------------------------------------------- 
    gripper_action("visor")
    point_zero = [
        center_c[0] + (radius_c + 30) * math.cos(math.radians(start_angle_v)), 
        center_c[1],  
        center_c[2] + (radius_c + 30) * math.sin(math.radians(start_angle_v)),  
        orientation_v[0], orientation_v[1], orientation_v[2]  
    ]
    TM12.ptp(point_zero,50)
    
    point_0 = visor_locking_up_XXXL(center_v, radius_v+8, start_angle_v, orientation_v)
    point_1 = visor_motion_up_XXXL(center_v, radius_v, max_angle_deltar_v, deltar_v, num_points_v, start_angle_v, end_angle_v, orientation_v)
    check_tcp_coord(point_1)    
    TM12.exit('1')

    time.sleep(vision_timeout)
    confirm_movement(ui,"\nVisor motion up completed")

    #CHINGUARD OPENING------------------------------------------------------------------------
    point_2 = [
        center_c[0] + (radius_c + deltar) * math.cos(math.radians(start_angle_c)), 
        center_c[1],  
        center_c[2] + (radius_c + deltar) * math.sin(math.radians(start_angle_c)),  
        orientation_c[0], orientation_c[1], orientation_c[2]  
    ]
    point_2 = transfer(point_1, point_2, center_c, radius_c)
    gripper_action("open")
    point_2_5=chinguard_locking_XXXL(center_c, radius_c, deltar, start_angle_c, orientation_c)
    check_tcp_coord(point_2_5)
    gripper_action("close")
    time.sleep(gripping_timeout)
    point_3 = chinguard_motion_XXXL("up",center_c, radius_c, max_angle_deltar, deltar, num_points_c, start_angle_c, end_angle_c, orientation_c)
    check_tcp_coord(point_3)
    gripper_action("open")
    TM12.exit('1')
    
    time.sleep(vision_timeout)
    confirm_movement(ui,"\nChinguard motion up completed")

    #VISOR CLOSING------------------------------------------------------------------------
    point_4 = transfer(point_3, point_1, center_c, radius_c)
    gripper_action("visor")
    point_5 = visor_motion_down_XXXL(center_v, radius_v-2, deltar, num_points_v, start_angle_v, end_angle_v, orientation_v)
    check_tcp_coord(point_5)
    TM12.exit('1')
    
    time.sleep(vision_timeout)
    confirm_movement(ui,"\nVisor motion down completed")
    
    #CHINGUARD CLOSING------------------------------------------------------------------------
    point_6 = transfer(point_5, point_3, center_c, radius_c) 
    gripper_action("open")
    point6_5=chinguard_locking_XXXL(center_c, radius_c-5, deltar, end_angle_c, orientation_c)
    check_tcp_coord(point6_5)
    gripper_action("close")
    time.sleep(gripping_timeout)
    point_7 = chinguard_motion_XXXL("down",center_c, radius_c, max_angle_deltar, deltar, num_points_c, start_angle_c-2, end_angle_c, orientation_c)
    check_tcp_coord(point_7)
    gripper_action("open")
    TM12.exit('1')        

    time.sleep(vision_timeout)
    confirm_movement(ui,"\nChinguard motion down completed") 

    #VISOR CLOSING------------------------------------------------------------------------
        
    point_8 = transfer(point_2, point_1, center_c, radius_c)
    gripper_action("visor")
    point_9 = visor_motion_down_XXXL(center_v, radius_v-2, deltar, num_points_v, start_angle_v, end_angle_v, orientation_v)
    check_tcp_coord(point_9)
    TM12.exit('1')
    
    time.sleep(vision_timeout)
    confirm_movement(ui,"\nVisor motion down completed")

def sequence_2_XXXL(ui):

    #computation of motion parameters
    center_c, radius_c, deltar, max_angle_deltar, num_points_c , orientation_c, start_angle_c, end_angle_c = chinguard_motion_parameters_XXXL()
    center_v, radius_v, deltar_v, max_angle_deltar_v, num_points_v , orientation_v, start_angle_v, end_angle_v = visor_motion_parameters_XXXL()

    #VISOR OPENING-------------------------------------------------------------------------- 
    gripper_action("visor")
    point_zero = [
        center_c[0] + (radius_c + 20) * math.cos(math.radians(start_angle_v)), 
        center_c[1],  
        center_c[2] + (radius_c + 20) * math.sin(math.radians(start_angle_v)),  
        orientation_v[0], orientation_v[1], orientation_v[2]  
    ]
    TM12.ptp(point_zero,50)
    
    point_0 = visor_locking_up_XXXL(center_v, radius_v+8, start_angle_v, orientation_v)
    point_1 = visor_motion_up_XXXL(center_v, radius_v, max_angle_deltar_v, deltar_v, num_points_v, start_angle_v, end_angle_v, orientation_v)
    check_tcp_coord(point_1)    
    TM12.exit('1')

    time.sleep(vision_timeout)
    confirm_movement(ui,"\nVisor motion up completed")

    #CHINGUARD OPENING------------------------------------------------------------------------
    point_2 = [
        center_c[0] + (radius_c + deltar) * math.cos(math.radians(start_angle_c)), 
        center_c[1],  
        center_c[2] + (radius_c + deltar) * math.sin(math.radians(start_angle_c)),  
        orientation_c[0], orientation_c[1], orientation_c[2]  
    ]
    point_2 = transfer(point_1, point_2, center_c, radius_c)
    gripper_action("open")
    point_2_5=chinguard_locking_XXXL(center_c, radius_c, deltar, start_angle_c, orientation_c)
    check_tcp_coord(point_2_5)
    gripper_action("close")
    time.sleep(gripping_timeout)
    point_3 = chinguard_motion_XXXL("up",center_c, radius_c, max_angle_deltar, deltar, num_points_c, start_angle_c, end_angle_c, orientation_c)
    check_tcp_coord(point_3)
    gripper_action("open")
    TM12.exit('1')
    
    time.sleep(vision_timeout)
    confirm_movement(ui,"\nChinguard motion up completed")
    
    # CHINGUARD CLOSING---------------------------------------------------------------------
    point6_5=chinguard_locking_XXXL(center_c, radius_c-5, deltar, end_angle_c, orientation_c)
    check_tcp_coord(point6_5)
    gripper_action("close")
    time.sleep(gripping_timeout)
    point_7 = chinguard_motion_XXXL("down",center_c, radius_c, max_angle_deltar, deltar, num_points_c, start_angle_c-2, end_angle_c, orientation_c)
    check_tcp_coord(point_7)
    gripper_action("open")
    TM12.exit('1')        

    time.sleep(vision_timeout)
    confirm_movement(ui,"\nChinguard motion down completed")

    # VISOR CLOSING------------------------------------------------------------------------
        
    point_8 = transfer(point_2, point_1, center_c, radius_c)
    gripper_action("visor")
    point_9 = visor_motion_down_XXXL(center_v, radius_v-2, deltar, num_points_v, start_angle_v, end_angle_v, orientation_v)
    check_tcp_coord(point_9)
    TM12.exit('1')
    
    time.sleep(vision_timeout)
    confirm_movement(ui,"\nVisor motion down completed")

    


MOVEMENT_REGISTRY = {
    "sequence_1_M": sequence_1_M,
    "sequence_2_M": sequence_2_M,
    "sequence_1_XXXL": sequence_1_XXXL,
    "sequence_2_XXXL": sequence_2_XXXL
}
