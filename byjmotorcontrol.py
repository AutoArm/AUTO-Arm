import serial

import time

 

class StepperController:

    def __init__(self, port='COM7', baud_rate=115200):

        self.serial = serial.Serial(port, baud_rate, timeout=1)

        time.sleep(2)

       

    def move_motor(self, motor_number, steps):

        """

        Move specified motor number of steps

        motor_number: 1 or 2

        steps: positive for forward, negative for backward

        """

        command = f"move,{motor_number},{steps}\n"

        self.serial.write(command.encode())

        response = self.serial.readline().decode().strip()

        return response

   

    def power_motor(self, motor_number, state):

        """

        Control power to motors

        motor_number: 0 for both motors, 1 for motor1, 2 for motor2

        state: True to enable, False to disable

        """

        command = f"power,{motor_number},{1 if state else 0}\n"

        self.serial.write(command.encode())

        response = self.serial.readline().decode().strip()

        return response

       

    def close(self):

        self.serial.close()

 

if __name__ == "__main__":

    controller = StepperController()  # Adjust port as needed

   

    try:

        # Enable both motors

        print("Enabling motors...")

        controller.power_motor(0, True)

       

        # Move motor 1

        print("Moving motor 1...")

        controller.move_motor(1, 2048)

        time.sleep(2)

       

        # Disable only motor 1

        print("Disabling motor 1...")

        controller.power_motor(1, False)

       

        # Move motor 2

        print("Moving motor 2...")

        controller.move_motor(2, -1*2048)

 

       

        # Disable all motors

        print("Disabling all motors...")

        controller.power_motor(0, False)

       

    finally:

        controller.close()