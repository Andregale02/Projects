import customtkinter as ctk
from tkinter import scrolledtext
import tkinter as tk
import threading
from constants import *
from gripper import *
from motion_sequences import MOVEMENT_REGISTRY

# In this file the user interface is designed, so to be able to:
# - choose the helmet size
# - choose the sequence 
# - start and end the requence
# - count the cycles 
# - display errors when they occur
# - display each completed motion step

import time
class UI(ctk.CTk):
    def __init__(self): # This function contains the setup of the UI window
        super().__init__()
        
        self.title("Helmet Motion Robot Control")
        self.geometry("900x700")
        self.minsize(800, 600)
        
        # Starting state
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
        
        # Create labels
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

        # Headers of possible options
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
    
    def select_helmet(self, helmet_size): # Action to perform when an helmet selection button is pressed
        """Handle helmet selection"""
        self.current_helmet = helmet_size
        self.sequence_running = False
        
        # Update button states so that selection choice becomes available
        self.M_button.configure(state="disabled")
        self.XXXL_button.configure(state="disabled")
        self.sequence1_button.configure(state="normal")
        self.sequence2_button.configure(state="normal")
        self.sequence_start_button.configure(state="disabled")
        self.sequence_end_button.configure(state="disabled")
        
        # Update status and log
        self.current_sequence = None
        self.status_indicator.configure(text="● Helmet Selected", text_color="#4ec9b0")
        self.add_log(f"Helmet {helmet_size} selected\n")
        self.add_log("Please select a sequence\n")
    
    def select_sequence(self, sequence_num): # Action to perfrom when a sequence is selected
        """Handle sequence selection"""
        self.current_sequence = sequence_num
        self.sequence_running = False
        
        # Update button states so that the control buttons (start and end sequence) become available
        self.sequence1_button.configure(state="disabled")
        self.sequence2_button.configure(state="disabled")
        self.sequence_start_button.configure(state="normal")
        self.sequence_end_button.configure(state="disabled")
        
        # Update status and log
        self.status_indicator.configure(text="● Sequence Selected", text_color="#4ec9b0")
        self.add_log(f"Sequence {sequence_num} selected for Helmet {self.current_helmet}\n")
        self.add_log("Press 'Start Sequence' to begin\n")
    
    def start_sequence(self):
        """Handle sequence start"""

        # Movement is started in a separate thread to keep the UI responsive, so before starting we check for other threads running
        if self.movement_thread and self.movement_thread.is_alive():
            return  # Prevent multiple threads
        
        # Update status
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
        
        # Update status and log
        self.status_indicator.configure(text="● Sequence Running", text_color="#d15c35")
        self.add_log(f"Sequence {self.current_sequence} started for Helmet {self.current_helmet}\n")
        self.add_log("Press 'End Sequence' to stop\n")
    
    def execute_movement(self):
        """Execute the movement pattern in a separate thread"""

        # Build the variable containing the action to perform
        selected_movement = f"sequence_{self.current_sequence}_{self.current_helmet}"
        
        # Connect to the first listen node, which exit defines the sequence to be performed in the flow
        TM12.connect_listen_node()
        time.sleep(5)

        # Choose from the movement registry the action to be performed
        movement_func = MOVEMENT_REGISTRY[selected_movement]
        
        while not self.stop_event.is_set():
            # open gripper communication
            port_handler.openPort()
            port_handler.setBaudRate(BAUDRATE)

            # exit the first listen (to define which sequence to perform in the flow)
            TM12.exit(f"{self.current_sequence-1}")

            # launch movement function
            movement_func(self)

            # update and display cycle count
            self.cycle_count += 1
            self.after(0, lambda: self.cycle_counter_label.configure(text=str(self.cycle_count)))

            # Close gripper communication
            port_handler.closePort()
        
        self.after(0, self.end_sequence)


    def end_sequence(self):
        """Handle sequence end"""

        # When the end sequence button is pressed, update the state
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
        

        
        # Update status and log
        self.status_indicator.configure(text="● System Ready", text_color="#4ec9b0")
        self.add_log(f"\nCompleted {self.cycle_count} cycles\n")
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
