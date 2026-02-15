# This file contains the function which define the motion parameters for each helmet and thir respective component, 
# since two different movement path are defined for chinguard and visor  


def chinguard_motion_parameters_M():

    # Define circle parameters (all units in mm)
    radius = 323  
    center_x = 0  
    center_y = -650    
    center_z = 515 
    center = [center_x, center_y, center_z]

    # Radius variation for the first part of the path, since the motion to be performed is not exactly a circumference
    delta_r = 5
    max_angle_deltar = -15

    # Start and end angular positions, defined on the circumference, where zero angle corresponds to the horizontal
    start_angle_chinguard = -52
    end_angle_chinguard = 158    # 158° 

    # Computation of the number of waypoints defining the robot motion
    num_points = int(round(end_angle_chinguard - start_angle_chinguard)/5)     # 5° increments

    # Orientation of the end effector
    rx = 0  
    ry = 90 + start_angle_chinguard
    rz = 180
    orientation = [rx, ry, rz]

    return center, radius, delta_r, max_angle_deltar, num_points , orientation, start_angle_chinguard, end_angle_chinguard

def visor_motion_parameters_M():

    # Define circle parameters (all units in mm)
    radius = 285  
    center_x = 0  
    center_y = -650 
    center_z = 515  
    center = [center_x, center_y, center_z]

    # Radius variation along the path, since visor motion is not exactly an arc of circumference
    delta_r = 15
    max_angle_deltar = 30

    # Start and end angular positions, as for the chinguard
    start_angle_visor = -9  
    end_angle_visor = 30

    # Computation of the number of waypoints
    num_points = int(round(end_angle_visor - start_angle_visor)/5)     #1° increments

    #Orientation of the end effector
    rx = 0  
    ry = 90 + start_angle_visor
    rz = 180
    orientation = [rx, ry, rz]

    return center, radius, delta_r, max_angle_deltar, num_points , orientation, start_angle_visor, end_angle_visor

def chinguard_motion_parameters_XXXL():

    # Define circle parameters (all units in mm)
    radius = 330  
    center_x = 0  
    center_y = -650    
    center_z = 515  
    center = [center_x, center_y, center_z]

    # Radius variation along the path 
    delta_r = 5
    max_angle_deltar = -15

    # Start and end angular positions
    start_angle_chinguard = -50
    end_angle_chinguard = 158

    # Computation of the number of waypoints to be followed along the path
    num_points = int(round(end_angle_chinguard - start_angle_chinguard)/5)     # 5° increments

    # Orientation of the end effector
    rx = 0  
    ry = 90 + start_angle_chinguard
    rz = 180
    orientation = [rx, ry, rz]
    return center, radius, delta_r, max_angle_deltar, num_points , orientation, start_angle_chinguard, end_angle_chinguard

def visor_motion_parameters_XXXL():

    # Define circle parameters (all units in mm)
    radius = 280  
    center_x = 5  
    center_y = -650
    center_z = 515
    center = [center_x, center_y, center_z]

    # Radius variation along the the path 
    delta_r = 5
    max_angle_deltar = -15

    # Start and end angular positions
    start_angle_visor = -9  
    end_angle_visor = 30   

    # Computation of the number of waypoints to be followed along the path
    num_points = int(round(end_angle_visor - start_angle_visor)/5)     #5° increments

    #Orientation of the end effector
    rx = 0  
    ry = 90 + start_angle_visor
    rz = 180
    orientation = [rx, ry, rz]

    return center, radius, delta_r, max_angle_deltar, num_points , orientation, start_angle_visor, end_angle_visor

