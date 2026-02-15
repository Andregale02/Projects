import math
import time
from constants import *
import numpy as np

# In this functions all the possible motion steps are defined, together with some auxiliary functions 

# PS: only the functions regarding helmet M are commented, as the one for helmet XXXL are exactly the same with few numerical differences

def chinguard_locking_M(center, radius, delta_r, angular_pos, orientation):
    locking_waypoints = []

    # Definition of the necessary waypoints to unlock the chinguard

    # starting further and rotated 
    locking_1 = [ 
        center[0] + (radius + delta_r + 10) * math.cos(math.radians(angular_pos)),  
        center[1],  
        center[2] + (radius + delta_r + 10) * math.sin(math.radians(angular_pos)),  
        orientation[0], 
        90 + angular_pos + 2.5,
        orientation[2]
    ]
    locking_waypoints.append(locking_1)

    # Moving towards the hemet 
    locking_2 = [
        center[0] + (radius + delta_r - 5) * math.cos(math.radians(angular_pos)),  
        locking_1[1],
        center[2] + (radius + delta_r - 5) * math.sin(math.radians(angular_pos)),  
        locking_1[3], 
        locking_1[4] + 2.5,
        locking_1[5]
    ]
    locking_waypoints.append(locking_2)

    # Rotation to pull back the lower part of the mechanism
    locking_3 = [
        locking_2[0],
        locking_2[1],
        locking_2[2],
        locking_2[3],
        locking_2[4] - 5,
        locking_2[5]
    ]
    locking_waypoints.append(locking_3)

    # Pull back
    locking_4 = [
        center[0] + (radius + delta_r) * math.cos(math.radians(angular_pos)),
        locking_3[1],
        center[2] + (radius + delta_r) * math.sin(math.radians(angular_pos)),
        locking_3[3],
        locking_3[4],
        locking_3[5]
    ]
    locking_waypoints.append(locking_4)
    
    

    # Motion
    for point in locking_waypoints[0:2]:
        TM12.ptp(point, 20)  
        time.sleep(2)    

    # Last movement a little slower
    TM12.ptp(locking_4, 10)

    return locking_4

def visor_locking_up_M(center, radius, angular_pos, orientation):
    locking_waypoints = []

    # Definition of the necessary waypoints to unlock the chinguard

    # Starting further and rotated
    locking_1 = [
        center[0] + (radius + 15) * math.cos(math.radians(angular_pos)),  
        center[1],  
        center[2] + (radius + 15) * math.sin(math.radians(angular_pos)),  
        orientation[0], 
        115 + angular_pos,
        orientation[2]
    ]
    locking_waypoints.append(locking_1)

    # Moving a bit closer without further rotation
    locking_2 = [
        center[0] + (radius + 10) * math.cos(math.radians(angular_pos)),  
        center[1],  
        center[2] + (radius + 10) * math.sin(math.radians(angular_pos)),  
        orientation[0], 
        115 + angular_pos,
        orientation[2]
    ]
    locking_waypoints.append(locking_2)

    # Rotation
    locking_3 = [
        center[0] + (radius+ 10) * math.cos(math.radians(angular_pos)),  
        locking_1[1],
        center[2] + (radius+ 10) * math.sin(math.radians(angular_pos)),  
        locking_1[3], 
        100 + angular_pos,
        locking_1[5]
    ]
    locking_waypoints.append(locking_3)

    # Moving towards the helmet
    locking_4 = [
        center[0] + (radius) * math.cos(math.radians(angular_pos)),  
        locking_1[1],
        center[2] + (radius) * math.sin(math.radians(angular_pos)),  
        locking_1[3], 
        100 + angular_pos,
        locking_1[5]
    ]
    locking_waypoints.append(locking_4)

    TM12.ptp(locking_1, 20)

    # Actual motion
    for point in locking_waypoints[1:]:
        TM12.ptp(point, 50)  
        time.sleep(2)    
    return point

def chinguard_motion_M(direction, center, radius, max_angle_deltar, delta_r, num_points, start_angle_chinguard, end_angle_chinguard, orientation):
    waypoints = []
    
    # Waypoints computation
    for i in range(num_points + 1):
        
        # Computation of the angular position for each point
        angle = start_angle_chinguard + (end_angle_chinguard - start_angle_chinguard) * (i / num_points)

        # If angular position smaller than threshold, change of circumference radijus along the path linearly, proportionally to the angular position
        if angle < max_angle_deltar:
            
            angle_rate = abs((max_angle_deltar - angle)/(max_angle_deltar-start_angle_chinguard))

            x = center[0] + (radius + delta_r*angle_rate) * math.cos(math.radians(angle))
            z = center[2] + (radius + delta_r*angle_rate) * math.sin(math.radians(angle))
            ry = angle + 90  
        else:
        
            x = center[0] + radius * math.cos(math.radians(angle))
            z = center[2] + radius * math.sin(math.radians(angle))
            ry = angle + 90  
        
        
        waypoint = [
            x,  
            center[1],         
            z,        
            orientation[0],   
            ry,        
            orientation[2]         
        ]
        waypoints.append(waypoint)

    # Distinction between upward and backward movement
    if direction == "up":
        for point in waypoints:
            TM12.ptp(point, 50)  
            time.sleep(0.05)    
        return waypoints[-1]
    if direction == "down":
        for point in waypoints[::-1]:
            TM12.ptp(point, 50)  
            time.sleep(0.05)    
        return waypoints[0]

def visor_motion_up_M(center_visor, radius_visor, max_angle_deltar, delta_r, num_points, start_angle_visor, end_angle_visor, orientation):
    
    # Fine tuning of the radius parameter
    radius_visor = radius_visor-10
    waypoints = []

    # Waypoints computation
    for i in range(num_points + 1):

        angle = start_angle_visor + (end_angle_visor - start_angle_visor) * (i / num_points)

        # If angular position smaller than threshold, change of circumference radijus along the path linearly, proportionally to the angular position
        if angle < max_angle_deltar:

            angle_rate = abs((max_angle_deltar - angle)/(max_angle_deltar-start_angle_visor))

            x = center_visor[0] + (radius_visor + delta_r*angle_rate) * math.cos(math.radians(angle))
            z = center_visor[2] + (radius_visor + delta_r*angle_rate) * math.sin(math.radians(angle))
            ry = angle + 100  
        else:

            x = center_visor[0] + radius_visor * math.cos(math.radians(angle))
            z = center_visor[2] + radius_visor * math.sin(math.radians(angle))
            ry = angle + 100  
        
        waypoint = [
            x,  
            center_visor[1],         
            z,        
            orientation[0],   
            ry,        
            orientation[2]         
        ]
        waypoints.append(waypoint)

    # Actual movement
    for point in waypoints:
        TM12.ptp(point, 50)  
        time.sleep(0.05)    
    return waypoints[-1]

def visor_motion_down_M(center_visor, radius_visor, delta_r, num_points, start_angle_visor, end_angle_visor, orientation):
    
    # Fine tuning of some parameter
    delta_r = 30
    radius_visor = radius_visor-13
    start_angle_visor = start_angle_visor - 3
    end_angle_visor = end_angle_visor +15
    
    waypoints = []

    # Waypoints computation
    for i in range(num_points + 1):

        # Radius linear variation along the whole path
        angle = start_angle_visor + (end_angle_visor - start_angle_visor) * (i / num_points)
        
        angle_rate = abs((angle - end_angle_visor)/(end_angle_visor-start_angle_visor))

        x = center_visor[0] + (radius_visor + delta_r*angle_rate) * math.cos(math.radians(angle))
        z = center_visor[2] + (radius_visor + delta_r*angle_rate) * math.sin(math.radians(angle))

        ry = angle + 125 
    
        
        waypoint = [
            x,  
            center_visor[1],         
            z,        
            orientation[0],   
            ry,        
            orientation[2]         
        ]
        waypoints.append(waypoint)

    # actial movement
    for point in waypoints[::-1]:
        TM12.ptp(point, 50)  
        time.sleep(0.05)    
    return waypoints[0]

#------------------- other helmet --------------------------------------------------------

def chinguard_locking_XXXL(center, radius, delta_r, angular_pos, orientation):
    radius=radius+2
    locking_waypoints = []

    locking_1 = [
        center[0] + (radius + delta_r + 10) * math.cos(math.radians(angular_pos)),  
        center[1],  
        center[2] + (radius + delta_r + 10) * math.sin(math.radians(angular_pos)),  
        orientation[0], 
        90 + angular_pos + 2.5,
        orientation[2]
    ]
    locking_waypoints.append(locking_1)

    locking_2 = [
        center[0] + (radius + delta_r - 5) * math.cos(math.radians(angular_pos)),  
        locking_1[1],
        center[2] + (radius + delta_r - 5) * math.sin(math.radians(angular_pos)),  
        locking_1[3], 
        locking_1[4] + 2.5,
        locking_1[5]
    ]
    locking_waypoints.append(locking_2)

    locking_3 = [
        locking_2[0],
        locking_2[1],
        locking_2[2],
        locking_2[3],
        locking_2[4] - 5,
        locking_2[5]
    ]
    locking_waypoints.append(locking_3)

    locking_4 = [
        center[0] + (radius + delta_r) * math.cos(math.radians(angular_pos)),
        locking_3[1],
        center[2] + (radius + delta_r) * math.sin(math.radians(angular_pos)),
        locking_3[3],
        locking_3[4],
        locking_3[5]
    ]
    locking_waypoints.append(locking_4)
    
    TM12.ptp(locking_1, 30)

    
    for point in locking_waypoints[1:]:
        TM12.ptp(point, 30)  
        time.sleep(2)    
    return point

def visor_locking_up_XXXL(center, radius, angular_pos, orientation):
        
    locking_waypoints = []

    locking_1 = [
        center[0] + (radius + 15) * math.cos(math.radians(angular_pos)),  
        center[1],  
        center[2] + (radius + 15) * math.sin(math.radians(angular_pos)),  
        orientation[0], 
        115 + angular_pos,
        orientation[2]
    ]
    locking_waypoints.append(locking_1)

    locking_2 = [
        center[0] + (radius + 10) * math.cos(math.radians(angular_pos)),  
        center[1],  
        center[2] + (radius + 10) * math.sin(math.radians(angular_pos)),  
        orientation[0], 
        115 + angular_pos,
        orientation[2]
    ]
    locking_waypoints.append(locking_2)

    locking_3 = [
        center[0] + (radius+ 10) * math.cos(math.radians(angular_pos)),  
        locking_1[1],
        center[2] + (radius+ 10) * math.sin(math.radians(angular_pos)),  
        locking_1[3], 
        100 + angular_pos,
        locking_1[5]
    ]
    locking_waypoints.append(locking_3)

    locking_4 = [
        center[0] + (radius) * math.cos(math.radians(angular_pos)),  
        locking_1[1],
        center[2] + (radius) * math.sin(math.radians(angular_pos)),  
        locking_1[3], 
        100 + angular_pos,
        locking_1[5]
    ]
    locking_waypoints.append(locking_4)

    TM12.ptp(locking_1, 20)

    
    for point in locking_waypoints[1:]:
        TM12.ptp(point, 50)  
        time.sleep(2)    
    return point

def chinguard_motion_XXXL(direction, center, radius, max_angle_deltar, delta_r, num_points, start_angle_chinguard, end_angle_chinguard, orientation):
    waypoints = []
    for i in range(num_points + 1):

        angle = start_angle_chinguard + (end_angle_chinguard - start_angle_chinguard) * (i / num_points)

        if angle < max_angle_deltar:

            angle_rate = abs((max_angle_deltar - angle)/(max_angle_deltar-start_angle_chinguard))

            x = center[0] + (radius + delta_r*angle_rate) * math.cos(math.radians(angle))
            z = center[2] + (radius + delta_r*angle_rate) * math.sin(math.radians(angle))
            ry = angle + 90  
        else:
        
            x = center[0] + radius * math.cos(math.radians(angle))
            z = center[2] + radius * math.sin(math.radians(angle))
            ry = angle + 90  
        
        
        waypoint = [
            x,  
            center[1],         
            z,        
            orientation[0],   
            ry,        
            orientation[2]         
        ]
        waypoints.append(waypoint)

    
    if direction == "up":
        for point in waypoints:
            TM12.ptp(point, 50)  
            time.sleep(0.05)    
        return waypoints[-1]
    if direction == "down":
        for point in waypoints[::-1]:
            TM12.ptp(point, 50)  
            time.sleep(0.05)    
        return waypoints[0]

def visor_motion_up_XXXL(center_visor, radius_visor, max_angle_deltar, delta_r, num_points, start_angle_visor, end_angle_visor, orientation):
    end_angle_visor
    waypoints = []
    for i in range(num_points + 1):

        angle = start_angle_visor + (end_angle_visor - start_angle_visor) * (i / num_points)

        if angle < max_angle_deltar:

            angle_rate = abs((max_angle_deltar - angle)/(max_angle_deltar-start_angle_visor))

            x = center_visor[0] + (radius_visor + delta_r*angle_rate) * math.cos(math.radians(angle))
            z = center_visor[2] + (radius_visor + delta_r*angle_rate) * math.sin(math.radians(angle))
            ry = angle + 100  
        else:

            x = center_visor[0] + radius_visor * math.cos(math.radians(angle))
            z = center_visor[2] + radius_visor * math.sin(math.radians(angle))
            ry = angle + 100  
        
        waypoint = [
            x,  
            center_visor[1],         
            z,        
            orientation[0],   
            ry,        
            orientation[2]         
        ]
        waypoints.append(waypoint)


    for point in waypoints:
        TM12.ptp(point, 50)  
        time.sleep(0.05)    
    return waypoints[-1]

def visor_motion_down_XXXL(center_visor, radius_visor, delta_r, num_points, start_angle_visor, end_angle_visor, orientation):

    #parameters refinement
    delta_r = 22
    start_angle_visor = start_angle_visor - 3
    end_angle_visor = end_angle_visor + 15
    waypoints = []
    for i in range(num_points + 1):

        angle = start_angle_visor + (end_angle_visor - start_angle_visor) * (i / num_points)
        
        angle_rate = abs((end_angle_visor - angle)/(end_angle_visor-start_angle_visor))

        x = center_visor[0] + (radius_visor + delta_r*angle_rate) * math.cos(math.radians(angle))
        z = center_visor[2] + (radius_visor + delta_r*angle_rate) * math.sin(math.radians(angle))

        ry = angle + 125 
    
        
        waypoint = [
            x,  
            center_visor[1],         
            z,        
            orientation[0],   
            ry,        
            orientation[2]         
        ]
        waypoints.append(waypoint)


    for point in waypoints[::-1]:
        TM12.ptp(point, 50)  
        time.sleep(0.05)    
    return waypoints[0]

#%% SUPPLEMENTARY FUNCTIONS----------------------------------------------------------------

def transfer(initial_point, final_point, center, radius): 
    # Transfer from a point to another along a circular path having radius 5 cm bigger than the chinguard one  
    radius = radius + 50

    # Comuputation of the number of wayponts
    num_points = int(round(abs(final_point[4]-initial_point[4])/5))
    
    waypoints = []

    # Waypoints computation
    for i in range(num_points + 1):

        angle = initial_point[4] - 92 + (final_point[4] - initial_point[4]) * (i / num_points)

        x = center[0] + radius * math.cos(math.radians(angle))
        z = center[2] + radius * math.sin(math.radians(angle))
        ry = angle + 90  

        # First waypoint is defined as an intermediate point in order to avoid the impact with the helmet
        if i == 0:
            x = center[0] + (radius-50) * math.cos(math.radians(angle))
            z = center[2] + (radius-50) * math.sin(math.radians(angle))
            ry = angle + 90  
        else:
            x = center[0] + radius * math.cos(math.radians(angle))
            z = center[2] + radius * math.sin(math.radians(angle))
            ry = angle + 90  

        waypoint = [
            x,  
            center[1],         
            z,        
            initial_point[3],   
            ry,        
            initial_point[5]         
        ]
        waypoints.append(waypoint)

    # The movement to the first two waypoints is slower so that the robot is more precise and avoids the collision with the helmet
    for point in waypoints[0:2]:
        TM12.ptp(point, 20)  
        time.sleep(0.05)   
    
    # Actual movement
    for point in waypoints[2:]:
        TM12.ptp(point, 50)  
        time.sleep(0.05)
    
    return waypoints[-1]

def check_tcp_coord(position,offset=np.array([0,0,0,0,0,0])):
    # This function checks whether the end effector has reached or not the desired position (with tolerance on the orientation), 
    # so to give the signal to start the vision job or operate the gripper
    while (((np.round(TM12.tcp_coord[0:2], decimals=1)+offset[0:2])==np.round(position[0:2], decimals=1)).all())  == False:
        #print((np.round(TM12.tcp_coord, decimals=1)+offset),np.round(position, decimals=1))
        time.sleep(0.1)
    time.sleep(0.5)

def confirm_movement(ui,text):
    # This function confirms whether the movement has been executed or not, by chacking if the TMflow project is still running 
    # or not (since if the vision fails the flow stops) by cheching the variable "Project_Run" which directly comes from the 
    # flow 
    tic = time.time()
    while(time.time()-tic < vision_timeout):
        check = TM12.TMSVR.state["Project_Run"][1]
    
    # If the project is still running, the log is updated
    if check == True:
        ui.after(0, ui.add_log, text)
    else: # Else, error message and sequence end
        ui.after(0, ui.add_log, "\nERROR PERFORMING THE MOVEMENT")
        ui.end_sequence()
