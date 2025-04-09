# GUI motor controller for FAT
import tkinter as tk
from tkinter import ttk
import serial
import time

class StepperController:
    def __init__(self, port='COM12', baud_rate=115200):

        self.serial = serial.Serial(port, baud_rate, timeout=1)
        time.sleep(2)  # Allow time for Arduino to reset
       
    def move_motor(self, motor_number, steps):
        """
        Move specified motor number of steps
        motor_number: 1, 2, or 3
        steps: positive for forward, negative for backward
        """
        command = f"move,{motor_number},{steps}\n"
        self.serial.write(command.encode())
        response = self.serial.readline().decode().strip()
        return response
   
    def move_all_motors(self, steps1, steps2, steps3):
        """
        Move all three motors simultaneously with different step counts
        steps1, steps2, steps3: Number of steps for each motor
                                (positive for forward, negative for backward)
        """
        command = f"moveall,{steps1},{steps2},{steps3}\n"
        self.serial.write(command.encode())
        response = self.serial.readline().decode().strip()
        return response
   
    def power_motor(self, motor_number, state):
        """
        Control power to motors
        motor_number: 0 for all motors, 1 for motor1, 2 for motor2, 3 for motor3
        state: True to enable, False to disable
        """
        command = f"power,{motor_number},{1 if state else 0}\n"
        self.serial.write(command.encode())
        response = self.serial.readline().decode().strip()
        return response
       
    def close(self):
        """Close the serial connection"""
        self.serial.close()

def create_motor_control_gui(controller):
    """
    Create and run a GUI for controlling stepper motors using directional buttons
   
    Args:
        controller: An instance of StepperController
       
    Returns:
        None (exits when the GUI is closed)
    """
    # Create main window
    root = tk.Tk()
    root.title("Stepper Motor Control")
    root.geometry("500x500")
    root.resizable(False, False)
   
    # Motor configuration frame
    config_frame = ttk.LabelFrame(root, text="Motor Configuration")
    config_frame.pack(padx=10, pady=10, fill="x")
   
    # Step size entries
    ttk.Label(config_frame, text="Motor 1 (Vertical) Steps:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
    motor1_steps = ttk.Entry(config_frame, width=10)
    motor1_steps.grid(row=0, column=1, padx=5, pady=5)
    motor1_steps.insert(0, "2048")
   
    ttk.Label(config_frame, text="Motor 3 (Horizontal) Steps:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
    motor3_steps = ttk.Entry(config_frame, width=10)
    motor3_steps.grid(row=1, column=1, padx=5, pady=5)
    motor3_steps.insert(0, "2048")
   
    # Power controls
    power_frame = ttk.LabelFrame(root, text="Power Control")
    power_frame.pack(padx=10, pady=10, fill="x")
   
    def toggle_power():
        if power_var.get():
            controller.power_motor(0, True)
            status_var.set("Motors: ENABLED")
        else:
            controller.power_motor(0, False)
            status_var.set("Motors: DISABLED")
   
    power_var = tk.BooleanVar(value=True)
    power_check = ttk.Checkbutton(power_frame, text="Enable Motors", variable=power_var, command=toggle_power)
    power_check.grid(row=0, column=0, padx=5, pady=5)
   
    # Initialize motors to enabled state
    controller.power_motor(0, True)
   
    # Status bar
    status_var = tk.StringVar(value="Motors: ENABLED")
    status_bar = ttk.Label(root, textvariable=status_var, relief="sunken", anchor="w")
    status_bar.pack(side="bottom", fill="x")
   
    # Direction controls frame
    dir_frame = ttk.LabelFrame(root, text="Direction Control")
    dir_frame.pack(padx=10, pady=10, expand=True)
   
    # Button size and padding
    btn_width = 3
    btn_padding = 5
   
    # Functions for button actions
    def move_up():
        steps = int(motor1_steps.get())
        status_var.set(f"Moving Motor 1 UP ({steps} steps)")
        controller.move_motor(1, steps)
   
    def move_down():
        steps = int(motor1_steps.get())
        status_var.set(f"Moving Motor 1 DOWN ({-steps} steps)")
        controller.move_motor(1, -steps)
   
    def move_left():
        steps = int(motor3_steps.get())
        status_var.set(f"Moving Motor 3 LEFT ({steps} steps)")
        controller.move_motor(3, steps)
   
    def move_right():
        steps = int(motor3_steps.get())
        status_var.set(f"Moving Motor 3 RIGHT ({-steps} steps)")
        controller.move_motor(3, -steps)
   
    def move_up_left():
        steps1 = int(motor1_steps.get())
        steps3 = int(motor3_steps.get())
        status_var.set(f"Moving UP-LEFT (Motor 1: {steps1}, Motor 3: {steps3})")
        controller.move_all_motors(steps1, 0, steps3)
   
    def move_up_right():
        steps1 = int(motor1_steps.get())
        steps3 = int(motor3_steps.get())
        status_var.set(f"Moving UP-RIGHT (Motor 1: {steps1}, Motor 3: {-steps3})")
        controller.move_all_motors(steps1, 0, -steps3)
   
    def move_down_left():
        steps1 = int(motor1_steps.get())
        steps3 = int(motor3_steps.get())
        status_var.set(f"Moving DOWN-LEFT (Motor 1: {-steps1}, Motor 3: {steps3})")
        controller.move_all_motors(-steps1, 0, steps3)
   
    def move_down_right():
        steps1 = int(motor1_steps.get())
        steps3 = int(motor3_steps.get())
        status_var.set(f"Moving DOWN-RIGHT (Motor 1: {-steps1}, Motor 3: {-steps3})")
        controller.move_all_motors(-steps1, 0, -steps3)
   
    # Create and place arrow buttons in a grid layout
    # Up-Left button
    btn_up_left = ttk.Button(dir_frame, text="↖", width=btn_width, command=move_up_left)
    btn_up_left.grid(row=0, column=0, padx=btn_padding, pady=btn_padding)
   
    # Up button
    btn_up = ttk.Button(dir_frame, text="↑", width=btn_width, command=move_up)
    btn_up.grid(row=0, column=1, padx=btn_padding, pady=btn_padding)
   
    # Up-Right button
    btn_up_right = ttk.Button(dir_frame, text="↗", width=btn_width, command=move_up_right)
    btn_up_right.grid(row=0, column=2, padx=btn_padding, pady=btn_padding)
   
    # Left button
    btn_left = ttk.Button(dir_frame, text="←", width=btn_width, command=move_left)
    btn_left.grid(row=1, column=0, padx=btn_padding, pady=btn_padding)
   
    # Center (empty)
    ttk.Label(dir_frame, text="", width=btn_width).grid(row=1, column=1)
   
    # Right button
    btn_right = ttk.Button(dir_frame, text="→", width=btn_width, command=move_right)
    btn_right.grid(row=1, column=2, padx=btn_padding, pady=btn_padding)
   
    # Down-Left button
    btn_down_left = ttk.Button(dir_frame, text="↙", width=btn_width, command=move_down_left)
    btn_down_left.grid(row=2, column=0, padx=btn_padding, pady=btn_padding)
   
    # Down button
    btn_down = ttk.Button(dir_frame, text="↓", width=btn_width, command=move_down)
    btn_down.grid(row=2, column=1, padx=btn_padding, pady=btn_padding)
   
    # Down-Right button
    btn_down_right = ttk.Button(dir_frame, text="↘", width=btn_width, command=move_down_right)
    btn_down_right.grid(row=2, column=2, padx=btn_padding, pady=btn_padding)
   
    # Style buttons to be larger
    for child in dir_frame.winfo_children():
        if isinstance(child, ttk.Button):
            child.configure(style="Large.TButton")
   
    # Create a large button style
    style = ttk.Style()
    style.configure("Large.TButton", font=("Arial", 14))
   
    # Exit button
    def exit_program():
        controller.power_motor(0, False)  # Disable all motors before exiting
        root.destroy()
   
    exit_btn = ttk.Button(root, text="Exit", command=exit_program)
    exit_btn.pack(pady=10)
   
    # Handle window close event
    root.protocol("WM_DELETE_WINDOW", exit_program)
   
    # Start the GUI event loop
    root.mainloop()

if __name__ == "__main__":
    # Create controller instance
    controller = StepperController()  # Adjust port as needed
   
    try:
        # Launch GUI
        create_motor_control_gui(controller)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Ensure controller is closed properly
        controller.close()
