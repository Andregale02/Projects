import techman as tm
import math
import numpy as np
import dynamixel_sdk as dxl
import tkinter as tk
import customtkinter as ctk
from tkinter import scrolledtext
import threading
import time

vision_timeout = 1  # seconds
gripping_timeout = 2  # seconds

#%% SERVOMOTOR SETUP AND GRIPPER FUNCTION--------------------------------------------------

DEVICENAME = 'COM7'
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0
DXL_ID = 1

# Memory adresses
ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POSITION = 36
ADDR_MOVING_SPEED = 32

# Conversion from ° to servomotor position
DEG_TO_POS = 1023 / 300  

# Communication setup
port_handler = dxl.PortHandler(DEVICENAME)
packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)

port_handler.openPort()
port_handler.setBaudRate(BAUDRATE)


# Set initial position to 0
current_position = 0
pos = int(current_position*DEG_TO_POS)
packet_handler.write2ByteTxRx(port_handler, DXL_ID, ADDR_GOAL_POSITION, pos)

print(f"Motore impostato alla posizione iniziale di {current_position}°")
time.sleep(2)


def gripper_action(action):

    #input("Press enter to operate the gripper")
    if action == "open":
        target_position = int(0 * DEG_TO_POS)
        packet_handler.write2ByteTxRx(port_handler, DXL_ID, ADDR_GOAL_POSITION, target_position)
        print(f"Nuova posizione: {target_position/DEG_TO_POS}°")
        time.sleep(1)

    if action == "close":
        target_position = int(260 * DEG_TO_POS)
        packet_handler.write2ByteTxRx(port_handler, DXL_ID, ADDR_GOAL_POSITION, target_position)
        print(f"Nuova posizione: {target_position/DEG_TO_POS}°")
        time.sleep(1)

    if action == "visor":
        target_position = int(190 * DEG_TO_POS)
        packet_handler.write2ByteTxRx(port_handler, DXL_ID, ADDR_GOAL_POSITION, target_position)
        print(f"Nuova posizione: {target_position/DEG_TO_POS}°")
        time.sleep(1)

#------------------------------------------------------------------------------------------

#%% UI-------------------------------------------------------------------------------------
class UI(ctk.CTk):
    def __init__(self):
        super().__init__()
        
        self.title("Helmet Motion Robot Control")
        self.geometry("900x700")
        self.minsize(800, 600)
        
        # Current state
        self.current_helmet = None
        self.current_sequence = None
        self.sequence_running = False
        self.movement_thread = None
        self.stop_event = threading.Event()
        
        # Grid layout
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        
        # Create header
        self.header = ctk.CTkFrame(self, corner_radius=0)
        self.header.grid(row=0, column=0, sticky="nsew")
        self.header.grid_columnconfigure(0, weight=1)
        
        self.title_label = ctk.CTkLabel(
            self.header, 
            text="Helmet Motion Robot Control",
            font=ctk.CTkFont(family="Segoe UI Variable", size=25, weight="bold")
        )
        self.title_label.grid(row=0, column=0, padx=20, pady=10, sticky="w")
        
        self.status_indicator = ctk.CTkLabel(
            self.header,
            text="● System Ready",
            text_color="#4ec9b0",
            font=ctk.CTkFont(family="Segoe UI Variable", size=20, weight="bold")
        )
        self.status_indicator.grid(row=0, column=1, padx=20, pady=10, sticky="e")
        
        # Main content
        self.main_frame = ctk.CTkFrame(self, corner_radius=0)
        self.main_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=(0,10))
        self.main_frame.grid_columnconfigure((0,1), weight=1)
        self.main_frame.grid_rowconfigure(0, weight=1)
        
        # Left panel: controls
        self.control_panel = ctk.CTkFrame(self.main_frame, corner_radius=8)
        self.control_panel.grid(row=0, column=0, sticky="nsew", padx=(0,10), pady=10)
        self.control_panel.grid_columnconfigure((0,1), weight=1)
        self.control_panel.grid_rowconfigure(6, weight=0)
        self.control_panel.grid_rowconfigure(7, weight=0)
        self.cycle_count = 0

        # Cycle counter
        ctk.CTkLabel(
            self.control_panel,
            text="Cycle Counter",
            font=ctk.CTkFont(family="Segoe UI Variable", size=20, weight="bold"),
            anchor="w"
        ).grid(row=6, column=0, columnspan=2, padx=20, pady=(15,5), sticky="ew")

        self.cycle_counter_label = ctk.CTkLabel(
            self.control_panel,
            text="0",
            font=ctk.CTkFont(family="Segoe UI Variable", size=40, weight="bold"),
            text_color="#4ec9b0"
        )
        self.cycle_counter_label.grid(row=7, column=0, columnspan=2, padx=20, pady=(0,15))

        # Panel title
        ctk.CTkLabel(
            self.control_panel,
            text="Helmet Selection",
            font=ctk.CTkFont(family="Segoe UI Variable", size=20, weight="bold"),
            anchor="w"
        ).grid(row=0, column=0, columnspan=2, padx=20, pady=(15,10), sticky="ew")

        ctk.CTkLabel(
            self.control_panel,
            text="Sequence Selection",
            font=ctk.CTkFont(family="Segoe UI Variable", size=20, weight="bold"),
            anchor="w"
        ).grid(row=2, column=0, columnspan=2, padx=20, pady=(15,10), sticky="ew")

        ctk.CTkLabel(
            self.control_panel,
            text="Sequence Control",
            font=ctk.CTkFont(family="Segoe UI Variable", size=20, weight="bold"),
            anchor="w"
        ).grid(row=4, column=0, columnspan=2, padx=20, pady=(15,10), sticky="ew")
                
        # Helmet buttons
        self.M_button = ctk.CTkButton(
            self.control_panel,
            text="Size M",
            font=ctk.CTkFont(family="Segoe UI Variable", size=20),
            corner_radius=6,
            height=40,
            command=lambda: self.select_helmet("M")
        )
        self.M_button.grid(row=1, column=0, padx=10, pady=8, sticky="nsew")
        
        self.XXXL_button = ctk.CTkButton(
            self.control_panel,
            text="Size XXXL",
            font=ctk.CTkFont(family="Segoe UI Variable", size=20),
            corner_radius=6,
            height=40,
            command=lambda: self.select_helmet("XXXL")
        )
        self.XXXL_button.grid(row=1, column=1, padx=10, pady=8, sticky="nsew")
        
        # Sequence buttons
        self.sequence1_button = ctk.CTkButton(
            self.control_panel,
            text="Sequence 1",
            font=ctk.CTkFont(family="Segoe UI Variable", size=20),
            corner_radius=6,
            height=40,
            state="disabled",
            command=lambda: self.select_sequence(1)
        )
        self.sequence1_button.grid(row=3, column=0, padx=10, pady=8, sticky="nsew")
        
        self.sequence2_button = ctk.CTkButton(
            self.control_panel,
            text="Sequence 2",
            font=ctk.CTkFont(family="Segoe UI Variable", size=20),
            corner_radius=6,
            height=40,
            state="disabled",
            command=lambda: self.select_sequence(2)
        )
        self.sequence2_button.grid(row=3, column=1, padx=10, pady=8, sticky="nsew")
        
        # Control buttons
        self.sequence_start_button = ctk.CTkButton(
            self.control_panel,
            text="Start Sequence",
            font=ctk.CTkFont(family="Segoe UI Variable", size=20),
            corner_radius=6,
            height=40,
            state="disabled",
            fg_color="#35d15c",
            hover_color="#35d15c",
            command=self.start_sequence
        )
        self.sequence_start_button.grid(row=5, column=0, padx=10, pady=8, sticky="nsew")
        
        self.sequence_end_button = ctk.CTkButton(
            self.control_panel,
            text="End Sequence",
            font=ctk.CTkFont(family="Segoe UI Variable", size=20),
            corner_radius=6,
            height=40,
            state="disabled",
            fg_color="#d13438",
            hover_color="#c12c30",
            command=self.end_sequence
        )
        self.sequence_end_button.grid(row=5, column=1, padx=10, pady=8, sticky="nsew")
        
        # Right panel - Status
        self.status_panel = ctk.CTkFrame(self.main_frame, corner_radius=8)
        self.status_panel.grid(row=0, column=1, sticky="nsew", pady=10)
        self.status_panel.grid_columnconfigure(0, weight=1)
        self.status_panel.grid_rowconfigure(1, weight=1)
        
        # Status panel title
        ctk.CTkLabel(
            self.status_panel,
            text="System Status",
            font=ctk.CTkFont(family="Segoe UI Variable", size=20, weight="bold"),
            anchor="w"
        ).grid(row=0, column=0, padx=20, pady=(15,10), sticky="ew")
        
        # Log text area 
        self.log_text = scrolledtext.ScrolledText(
            self.status_panel,
            wrap=tk.WORD,
            font=("Consolas", 20),
            fg="#ffffff",
            bg="#000000",
            insertbackground="#000000",
            padx=15,
            pady=15,
            relief="flat",
            borderwidth=0
        )
        self.log_text.grid(row=1, column=0, padx=10, pady=(0,15), sticky="nsew")
        
        # Add sample log content
        self.log_text.insert(tk.END, "Ready for helmet selection\n")
        self.log_text.insert(tk.END, "> _")
        self.log_text.configure(state="disabled")
        
        # Footer
        self.footer = ctk.CTkFrame(self, corner_radius=0, height=30)
        self.footer.grid(row=2, column=0, sticky="nsew")
        self.footer.grid_columnconfigure(0, weight=1)
        
        ctk.CTkLabel(
            self.footer,
            text="v1.0.0",
            text_color=("#7a7a7a", "#7a7a7a"),
            font=ctk.CTkFont(family="Segoe UI Variable", size=15),
            anchor="e"
        ).grid(row=0, column=0, padx=20, sticky="e")
    
    def select_helmet(self, helmet_size):
        """Handle helmet selection"""
        self.current_helmet = helmet_size
        self.sequence_running = False
        
        # Update button states
        self.M_button.configure(state="disabled")
        self.XXXL_button.configure(state="disabled")
        self.sequence1_button.configure(state="normal")
        self.sequence2_button.configure(state="normal")
        self.sequence_start_button.configure(state="disabled")
        self.sequence_end_button.configure(state="disabled")
        
        # Update status
        self.current_sequence = None
        self.status_indicator.configure(text="● Helmet Selected", text_color="#4ec9b0")
        self.add_log(f"Helmet {helmet_size} selected\n")
        self.add_log("Please select a sequence\n")
    
    def select_sequence(self, sequence_num):
        """Handle sequence selection"""
        self.current_sequence = sequence_num
        self.sequence_running = False
        
        # Update button states
        self.sequence1_button.configure(state="disabled")
        self.sequence2_button.configure(state="disabled")
        self.sequence_start_button.configure(state="normal")
        self.sequence_end_button.configure(state="disabled")
        
        # Update status
        self.status_indicator.configure(text="● Sequence Selected", text_color="#4ec9b0")
        self.add_log(f"Sequence {sequence_num} selected for Helmet {self.current_helmet}\n")
        self.add_log("Press 'Start Sequence' to begin\n")
    
    def start_sequence(self):
        """Handle sequence start"""
        if self.movement_thread and self.movement_thread.is_alive():
            return  # Prevent multiple threads
        
        self.cycle_count = 0
        self.cycle_counter_label.configure(text=str(self.cycle_count))
        self.sequence_running = True
        self.stop_event.clear()
        
        # Start movement in a separate thread
        self.movement_thread = threading.Thread(
            target=self.execute_movement,
            daemon=True
        )
        self.movement_thread.start()
        
        # Update button states
        self.sequence_start_button.configure(state="disabled")
        self.sequence_end_button.configure(state="normal")
        
        # Update status
        self.status_indicator.configure(text="● Sequence Running", text_color="#d15c35")
        self.add_log(f"Sequence {self.current_sequence} started for Helmet {self.current_helmet}\n")
        self.add_log("Press 'End Sequence' to stop\n")
    
    def execute_movement(self):
        """Execute the movement pattern in a separate thread"""
        selected_movement = f"sequence_{self.current_sequence}_{self.current_helmet}"
        #try:
        TM12.connect_listen_node()
        time.sleep(5)
        movement_func = MOVEMENT_REGISTRY[selected_movement]
        
        while not self.stop_event.is_set():
            port_handler.openPort()
            port_handler.setBaudRate(BAUDRATE)

            TM12.exit(f"{self.current_sequence-1}")
            movement_func(self)
            self.cycle_count += 1
            self.after(0, lambda: self.cycle_counter_label.configure(text=str(self.cycle_count)))

            port_handler.closePort()
        

        self.after(0, self.end_sequence)
        '''except KeyError:
            self.add_log(f"Error: Movement {selected_movement} not found\n")
            TM12.stop()
        except Exception as e:
            TM12.stop()
            self.add_log(f"Error in movement: {str(e)}\n")'''

    def end_sequence(self):
        """Handle sequence end"""
        self.stop_event.set()
        self.sequence_running = False
        
        # Wait for thread to finish if it's running
        if self.movement_thread and self.movement_thread.is_alive():
            self.movement_thread.join(timeout=1)
        
        # Update button states
        self.M_button.configure(state="normal")
        self.XXXL_button.configure(state="normal")
        self.sequence1_button.configure(state="disabled")
        self.sequence2_button.configure(state="disabled")
        self.sequence_start_button.configure(state="disabled")
        self.sequence_end_button.configure(state="disabled")
        #self.cycle_counter_label.configure(text="0")

        
        # Update status
        self.status_indicator.configure(text="● System Ready", text_color="#4ec9b0")
        self.add_log(f"\nCompleted {self.cycle_count} cycles\n")  # Add cycle count to log
        self.add_log(f"Sequence {self.current_sequence} completed for Helmet {self.current_helmet}\n")
        self.add_log("Ready for new helmet selection\n")
        
        # Reset selections
        self.current_helmet = None
        self.current_sequence = None
        self.cycle_count = 0
    
    def add_log(self, message):
        """Add message to log with proper formatting"""
        self.log_text.configure(state="normal")
        
        # Get all text including the prompt
        current_text = self.log_text.get("1.0", "end-1c")
        
        # Check if prompt exists at the end
        if current_text.endswith("> _"):
            # Remove the prompt by deleting from the start of the prompt to end
            prompt_start = current_text.rfind("> _")
            self.log_text.delete(f"1.0 + {prompt_start}c", "end-1c")
        
        # Add new message and new prompt
        self.log_text.insert("end", message)
        self.log_text.insert("end", "> _")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")
#------------------------------------------------------------------------------------------

#%% MOTION PARAMETERS----------------------------------------------------------------------
def chinguard_motion_parameters_M():
    # Define circle parameters (all units in mm)
    radius = 323  # 328
    center_x = 0  
    center_y = -650    # Y position of circle center (XZ plane)
    center_z = 515  # center height (Z)
    center = [center_x, center_y, center_z]

    # Radius variation for the first part of the path 
    delta_r = 5
    max_angle_deltar = -15

    #Start and end angular positions
    start_angle_chinguard = -52

    end_angle_chinguard = 158    # 158° 
    num_points = int(round(end_angle_chinguard - start_angle_chinguard)/5)     # 5° increments

    # Orientation
    rx = 0  
    ry = 90 + start_angle_chinguard
    rz = 180
    orientation = [rx, ry, rz]
    return center, radius, delta_r, max_angle_deltar, num_points , orientation, start_angle_chinguard, end_angle_chinguard

def visor_motion_parameters_M():
    # Define circle parameters (all units in mm)
    radius = 285  # TO BE TUNED
    center_x = 0  
    center_y = -650 # Y position of circle center (XZ plane)
    center_z = 515  # center height (Z)
    center = [center_x, center_y, center_z]

    # Radius variation for the first part of the path (IF NEEDED)
    delta_r = 15
    max_angle_deltar = 30

    #Start and end angular positions
    start_angle_visor = -9  # TO BE TUNED
    end_angle_visor = 30
        # TO BE TUNED 
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
    center_y = -650    # Y position of circle center (XZ plane)
    center_z = 515  # center height (Z)
    center = [center_x, center_y, center_z]

    # Radius variation for the first part of the path 
    delta_r = 5
    max_angle_deltar = -15

    #Start and end angular positions
    start_angle_chinguard = -50
    end_angle_chinguard = 158

    num_points = int(round(end_angle_chinguard - start_angle_chinguard)/5)     # 5° increments

    # Orientation
    rx = 0  
    ry = 90 + start_angle_chinguard
    rz = 180
    orientation = [rx, ry, rz]
    return center, radius, delta_r, max_angle_deltar, num_points , orientation, start_angle_chinguard, end_angle_chinguard

def visor_motion_parameters_XXXL():
    # Define circle parameters (all units in mm)
    radius = 280  # TO BE TUNED
    center_x = 5  
    center_y = -650 # Y position of circle center (XZ plane)
    center_z = 515  # center height (Z)
    center = [center_x, center_y, center_z]

    # Radius variation for the first part of the path (IF NEEDED)
    delta_r = 5
    max_angle_deltar = -15

    #Start and end angular positions
    start_angle_visor = -9  # TO BE TUNED
    end_angle_visor = 30    # TO BE TUNED 
    num_points = int(round(end_angle_visor - start_angle_visor)/5)     #5° increments

    #Orientation of the end effector
    rx = 0  
    ry = 90 + start_angle_visor
    rz = 180
    orientation = [rx, ry, rz]

    return center, radius, delta_r, max_angle_deltar, num_points , orientation, start_angle_visor, end_angle_visor

#------------------------------------------------------------------------------------------

#%% MOTION STEPS---------------------------------------------------------------------------
def chinguard_locking_M(center, radius, delta_r, angular_pos, orientation):
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
    

    #input("Press_enter to lock chinguard")

    for point in locking_waypoints[0:2]:
        #TM12.pline(point, 20)
        TM12.ptp(point, 20)  
        time.sleep(2)    

    TM12.ptp(locking_4, 10)

    return locking_4

def visor_locking_up_M(center, radius, angular_pos, orientation):
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

    #input("Press_enter to lock visor")

    for point in locking_waypoints[1:]:
        TM12.ptp(point,20)  
        time.sleep(2)    
    return point

def visor_locking_down_M(center, radius, angular_pos, orientation):
    locking_waypoints = []

    locking_1 = [
        center[0] + (radius) * math.cos(math.radians(angular_pos)),  
        center[1],  
        center[2] + (radius) * math.sin(math.radians(angular_pos)),  
        orientation[0], 
        90 + angular_pos,
        orientation[2]
    ]
    locking_waypoints.append(locking_1)

    

    TM12.ptp(locking_1, 20)

    #input("Press_enter to lock visor")

    for point in locking_waypoints[:]:
        TM12.pline(point,170)  
        time.sleep(2)    
    return point

def chinguard_motion_M(direction, center, radius, max_angle_deltar, delta_r, num_points, start_angle_chinguard, end_angle_chinguard, orientation):
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

    #print(waypoints[0:5])

    #input("Press enter to move chinguard")

    if direction == "up":
        for point in waypoints:
            TM12.pline(point,170)  
            time.sleep(0.05)    
        return waypoints[-1]
    if direction == "down":
        for point in waypoints[::-1]:
            TM12.pline(point,170)  
            time.sleep(0.05)    
        return waypoints[0]

def visor_motion_up_M(center_visor, radius_visor, max_angle_deltar, delta_r, num_points, start_angle_visor, end_angle_visor, orientation):
    radius_visor = radius_visor-10

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


    #input("Press enter to move visor")

    for point in waypoints:
        TM12.pline(point,170)  
        time.sleep(0.05)    
    return waypoints[-1]

def visor_motion_down_M(center_visor, radius_visor, delta_r, num_points, start_angle_visor, end_angle_visor, orientation):
    
    #parameters refinement
    delta_r = 30
    radius_visor = radius_visor-13
    start_angle_visor = start_angle_visor - 3
    end_angle_visor = end_angle_visor +15
    mean_angle = (end_angle_visor + start_angle_visor)/2

    waypoints = []
    for i in range(num_points + 1):

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


    #input("Press enter to move visor")

    for point in waypoints[::-1]:
        TM12.pline(point,170)  
        time.sleep(0.05)    
    return waypoints[0]

#------------------- other helmet --------------------------------------------------------

def chinguard_locking_XXXL(center, radius, delta_r, angular_pos, orientation):
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
    
    TM12.ptp(locking_1, 20)

    #input("Press_enter to lock chinguard")

    for point in locking_waypoints[1:]:
        
        TM12.ptp(point, 20)  
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

    #input("Press_enter to lock visor")

    for point in locking_waypoints[1:]:
        TM12.pline(point,170)  
        time.sleep(2)    
    return point

def visor_locking_down_XXXL(center, radius, angular_pos, orientation):
    locking_waypoints = []

    locking_1 = [
        center[0] + (radius) * math.cos(math.radians(angular_pos)),  
        center[1],  
        center[2] + (radius) * math.sin(math.radians(angular_pos)),  
        orientation[0], 
        115 + angular_pos,
        orientation[2]
    ]
    locking_waypoints.append(locking_1)

    

    TM12.ptp(locking_1, 50)

    input("Press_enter to lock visor")

    for point in locking_waypoints[:]:
        TM12.ptp(point, 30)  
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

    #print(waypoints[0:5])

    #input("Press enter to move chinguard")

    if direction == "up":
        for point in waypoints:
            TM12.pline(point,170)  
            time.sleep(0.05)    
        return waypoints[-1]
    if direction == "down":
        for point in waypoints[::-1]:
            TM12.pline(point,170)  
            time.sleep(0.05)    
        return waypoints[0]

def visor_motion_up_XXXL(center_visor, radius_visor, max_angle_deltar, delta_r, num_points, start_angle_visor, end_angle_visor, orientation):
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


    #input("Press enter to move visor")

    for point in waypoints:
        TM12.pline(point,170)  
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


    #input("Press enter to move visor")

    for point in waypoints[::-1]:
        TM12.pline(point,170)  
        time.sleep(0.05)    
    return waypoints[0]

#%% SUPPLEMENTARY FUNCTIONS----------------------------------------------------------------

def transfer(initial_point, final_point, center, radius):
    radius = radius + 50

    num_points = int(round(abs(final_point[4]-initial_point[4])/5))
    
    waypoints = []
    for i in range(num_points + 1):

        angle = initial_point[4] - 92 + (final_point[4] - initial_point[4]) * (i / num_points)

        x = center[0] + radius * math.cos(math.radians(angle))
        z = center[2] + radius * math.sin(math.radians(angle))
        ry = angle + 90  #max(angle + 90, 180)
        
        if i == 0:
            x = center[0] + (radius-50) * math.cos(math.radians(angle))
            z = center[2] + (radius-50) * math.sin(math.radians(angle))
            ry = angle + 90  #max(angle + 90, 180)
        else:
            x = center[0] + radius * math.cos(math.radians(angle))
            z = center[2] + radius * math.sin(math.radians(angle))
            ry = angle + 90  #max(angle + 90, 180)'''
    
        waypoint = [
            x,  
            center[1],         
            z,        
            initial_point[3],   
            ry,        
            initial_point[5]         
        ]
        waypoints.append(waypoint)

    #input("Press enter to move to the next point")
    
    for point in waypoints[0:2]:
        TM12.ptp(point, 70)  
        time.sleep(0.05)   
    
    for point in waypoints[2:]:
        TM12.pline(point,190)  
        time.sleep(0.05)
    
    return waypoints[-1]

def check_tcp_coord(position,offset=np.array([0,0,0,0,0,0])):
                while (((np.round(TM12.tcp_coord[0:2], decimals=1)+offset[0:2])==np.round(position[0:2], decimals=1)).all())  == False:
                    print((np.round(TM12.tcp_coord, decimals=1)+offset),np.round(position, decimals=1))
                    time.sleep(0.1)
                time.sleep(0.5)

def confirm_movement(ui,text):
    tic = time.time()
    while(time.time()-tic < vision_timeout):
        check = TM12.TMSVR.state["Project_Run"][1]
    
    if check == True:
        ui.after(0, ui.add_log, text)
    else:
        ui.after(0, ui.add_log, "\nERROR PERFORMING THE MOVEMENT")
        ui.end_sequence()

#------------------------------------------------------------------------------------------

#%% MOVEMENT-------------------------------------------------------------------------------
def sequence_1_M(ui):


    # Computation of motion parameters
    center_c, radius_c, deltar, max_angle_deltar, num_points_c , orientation_c, start_angle_c, end_angle_c = chinguard_motion_parameters_M()
    center_v, radius_v, deltar_v, max_angle_deltar_v, num_points_v , orientation_v, start_angle_v, end_angle_v = visor_motion_parameters_M()
    
    # VISOR OPENING------------------------------------------------------------------------
    gripper_action("visor")
    point_zero = [
        center_c[0] + (radius_c + 50) * math.cos(math.radians(start_angle_v)), 
        center_c[1],  
        center_c[2] + (radius_c + 50) * math.sin(math.radians(start_angle_v)),  
        orientation_v[0], orientation_v[1], orientation_v[2]  
    ]
    TM12.ptp(point_zero,20)

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

    #VISOR CLOSING------------------------------------------------------------------------
    
    point_4 = transfer(point_3, point_1, center_c, radius_c)
    gripper_action("visor")
    point_5 = visor_motion_down_M(center_v, radius_v, deltar, num_points_v, start_angle_v, end_angle_v, orientation_v)
    check_tcp_coord(point_5)
    TM12.exit('1')
    
    time.sleep(vision_timeout)
    confirm_movement(ui,"\nVisor motion down completed")

    #CHINGUARD CLOSING--------------------------------------------------------------------
    
    point_6 = transfer(point_5, point_3, center_c, radius_c) 
    gripper_action("open")
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

    #port_handler.closePort()

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

    '''#VISOR CLOSING----------------------------------------------------------------------
    point_4 = transfer(point_3, point_1, center_c, radius_c)
    gripper_action("visor")
    point_5 = visor_motion_down_M(center_v, radius_v, deltar, num_points_v, start_angle_v, end_angle_v, orientation_v)
    check_tcp_coord(point_5, offset=np.array([0,0,0,-180,46,180]))
    TM12.exit('1')
    
    time.sleep(vision_timeout)
    print("Visor motion completed")
    
    #CHINGUARD CLOSING----------------------------------------------------------------------
    gripper_action("open")
    point_6 = transfer(point_5, point_3, center_c, radius_c) '''
    
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

    #port_handler.closePort()

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
    point_2_5=chinguard_locking_XXXL(center_c, radius_c+2, deltar, start_angle_c, orientation_c)
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
    point6_5=chinguard_locking_XXXL(center_c, radius_c-3, deltar, end_angle_c, orientation_c)
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
    #TM12.ptp(point_2,10)
    
    point_8 = transfer(point_2, point_1, center_c, radius_c)
    gripper_action("visor")
    point_9 = visor_motion_down_XXXL(center_v, radius_v-2, deltar, num_points_v, start_angle_v, end_angle_v, orientation_v)
    check_tcp_coord(point_9)
    TM12.exit('1')
    
    time.sleep(vision_timeout)
    confirm_movement(ui,"\nVisor motion down completed")

    #port_handler.closePort()

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
    point_2_5=chinguard_locking_XXXL(center_c, radius_c+2, deltar, start_angle_c, orientation_c)
    check_tcp_coord(point_2_5)
    gripper_action("close")
    time.sleep(gripping_timeout)
    point_3 = chinguard_motion_XXXL("up",center_c, radius_c, max_angle_deltar, deltar, num_points_c, start_angle_c, end_angle_c, orientation_c)
    check_tcp_coord(point_3)
    gripper_action("open")
    TM12.exit('1')
    
    time.sleep(vision_timeout)
    confirm_movement(ui,"\nChinguard motion up completed")
    '''
    #VISOR CLOSING------------------------------------------------------------------------
    point_4 = transfer(point_3, point_1, center_c, radius_c)
    gripper_action("visor")
    point_5 = visor_motion_down_XXXL(center_v, radius_v-2, deltar, num_points_v, start_angle_v, end_angle_v, orientation_v)
    check_tcp_coord(point_5)
    TM12.exit('1')
    
    time.sleep(vision_timeout)
    ui.after(0, ui.add_log, "\nVisor motion down completed")
    
    #CHINGUARD CLOSING------------------------------------------------------------------------
    point_6 = transfer(point_5, point_3, center_c, radius_c) 
    gripper_action("open")'''
    point6_5=chinguard_locking_XXXL(center_c, radius_c-3, deltar, end_angle_c, orientation_c)
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
    #TM12.ptp(point_2,10)
    
    point_8 = transfer(point_2, point_1, center_c, radius_c)
    gripper_action("visor")
    point_9 = visor_motion_down_XXXL(center_v, radius_v-2, deltar, num_points_v, start_angle_v, end_angle_v, orientation_v)
    check_tcp_coord(point_9)
    TM12.exit('1')
    
    time.sleep(vision_timeout)
    confirm_movement(ui,"\nVisor motion down completed")

    #port_handler.closePort()



MOVEMENT_REGISTRY = {
    "sequence_1_M": sequence_1_M,
    "sequence_2_M": sequence_2_M,
    "sequence_1_XXXL": sequence_1_XXXL,
    "sequence_2_XXXL": sequence_2_XXXL
}
#------------------------------------------------------------------------------------------

TM12 = tm.TM_Robot("192.168.1.1")#127.0.0.1 192.168.1.1

if __name__ == "__main__":
    app = UI()
    app.mainloop()

    