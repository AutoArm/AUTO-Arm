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
 
if __name__ == "__main__":
    controller = StepperController()  # Adjust port as needed
   
    try:
        # Enable all motors
        print("Enabling motors...")
        controller.power_motor(0, True)
       
        # Test individual motors
        print("Moving motor 1...")
        controller.move_motor(1, 2048)  # Full rotation for 28BYJ-48 is ~4096 steps
        time.sleep(1)
       
        print("Moving motor 2...")
        controller.move_motor(2, 2048)
        time.sleep(1)
       
        print("Moving motor 3...")
        controller.move_motor(3, 2048)
        time.sleep(1)
       
        # Test simultaneous movement
        print("Moving all motors simultaneously...")
        controller.move_all_motors(2024, -2024, 2048)  # Different steps for each motor
       
        # Disable all motors when done
        print("Disabling all motors...")
        controller.power_motor(0, False)
       
    finally:
        controller.close()