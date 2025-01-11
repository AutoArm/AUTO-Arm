from xarm.wrapper import XArmAPI
import pyfirmata
import time
import sys
import math
import os
from configparser import ConfigParser

class RobotArm:
    def __init__(self, ip=None, arduino_port='COM4'):
        """
        Initialize the RobotArm object and configure the Arduino board for motor control.

        Parameters
        ----------
        ip : str, optional
            The IP address of the xArm. If not provided, the IP will be obtained from the command line
            or configuration file.
        arduino_port : str, optional
            The port where the Arduino is connected (Supposed to be 'COM4'). May change later.

        Returns
        -------
        None
        """
        # Set up the xArm 
        self.setup_xarm(ip) 
        # Set up the Arduino
        self.setup_arduino(arduino_port)
        self.start()

    def setup_xarm(self, ip=None):
        """
        Set up the xArm by initializing it with the correct IP address.
        If an IP address is not provided, it will attempt to retrieve it from
        command line arguments, a configuration file, or prompt the user.

        Parameters
        ----------
        ip : str, optional
            The IP address of the xArm.

        Returns
        -------
        None
        """
        sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
        if not ip:
            if len(sys.argv) >= 2:
                self.ip = sys.argv[1]
            else:
                try:
                    parser = ConfigParser()
                    parser.read('../robot.conf')
                    self.ip = parser.get('xArm', 'ip')
                except:
                    self.ip = input('Please input the xArm ip address:')
                    if not self.ip:
                        print('Input error, exiting.')
                        sys.exit(1)
        else:
            self.ip = ip

        self.arm = XArmAPI(self.ip)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        print("xArm setup done.")

    def setup_arduino(self, port):
        """
        Set up the Arduino board and configure pins for motor control.

        Parameters
        ----------
        port : str
            The port where the Arduino is connected.

        Returns
        -------
        None
        """
        self.board = pyfirmata.Arduino(port)

        # Configure pins for Motor 1
        self.pul_pin1 = 9  # Connect to PUL+ on DM542T
        self.dirp_pin1 = 7  # Connect to DIR+ on DM542T
        self.dirm_pin1 = 5  # Connect to DIR- on DM542T

        # Configure pins for Motor 2
        self.pul_pin2 = 4  # Connect to PUL+ on DM542T
        self.dirp_pin2 = 3  # Connect to DIR+ on DM542T
        self.dirm_pin2 = 2  # Connect to DIR- on DM542T

        self.ena_pin1 = None  # Optional pin for ENA+ on DM542T (Motor 1)
        self.ena_pin2 = None  # Optional pin for ENA+ on DM542T (Motor 2)

        # Set up the pins for Motor 1
        self.board.digital[self.pul_pin1].mode = pyfirmata.OUTPUT
        self.board.digital[self.dirp_pin1].mode = pyfirmata.OUTPUT
        self.board.digital[self.dirm_pin1].mode = pyfirmata.OUTPUT

        # Set up the pins for Motor 2
        self.board.digital[self.pul_pin2].mode = pyfirmata.OUTPUT
        self.board.digital[self.dirp_pin2].mode = pyfirmata.OUTPUT
        self.board.digital[self.dirm_pin2].mode = pyfirmata.OUTPUT

        # Enable Motor 1 (if pin is configured)
        if self.ena_pin1 is not None:
            self.board.digital[self.ena_pin1].mode = pyfirmata.OUTPUT
            self.board.digital[self.ena_pin1].write(0)  # Enable the driver (active LOW)

        # Enable Motor 2 (if pin is configured)
        if self.ena_pin2 is not None:
            self.board.digital[self.ena_pin2].mode = pyfirmata.OUTPUT
            self.board.digital[self.ena_pin2].write(0)  # Enable the driver (active LOW)

        self.steps_per_revolution = 200  # Typically 200 for a 1.8° stepper motor
        self.microstep_division = 1  # Set to your DM542T's microstep setting
        self.pulses_per_revolution = self.steps_per_revolution * self.microstep_division

        print("Arduino setup done.")

    def start(self):
        """
        Initializes the robot arm by setting the counter and setting the initial angle of the gripper.

        Returns
        -------
        None
        """
        try:
            # Install xArm Gripper
            print("Initializing additional settings for the xArm...")
            code = self.arm.set_counter_reset()
            print("Counter reset code:", code)
            weight = 0.610 
            center_of_gravity = (0.06125, 0.0458, 0.0375) 
            self.arm.set_tcp_load(weight=weight, center_of_gravity=center_of_gravity)
            code = self.arm.set_servo_angle(angle=[180, 75, -180, 20, 0, 90, -60], is_radian=False, speed=30, wait=True)
            print("xArm setup completed successfully.")

        except Exception as e:
            print(f'MainException: {e}')

    def rotate_motor(self, motor_id, revolutions, direction, speed_rpm=60):
        """
        Rotate the specified motor a specified number of revolutions.

        Parameters
        ----------
        motor_id : int
            Motor ID (1 for Motor 1, 2 for Motor 2).
        revolutions : float
            Number of revolutions (can be fractional).
        direction : bool
            True for one direction, False for the opposite.
        speed_rpm : int, optional
            Speed in rotations per minute (default is 60).

        Returns
        -------
        None
        """
        total_pulses = int(abs(revolutions) * self.pulses_per_revolution)
        delay = (60 / (speed_rpm * self.pulses_per_revolution)) / 2

        # Set pin configuration based on motor ID
        if motor_id == 1:
            pul_pin = self.pul_pin1
            dirp_pin = self.dirp_pin1
            dirm_pin = self.dirm_pin1
        elif motor_id == 2:
            pul_pin = self.pul_pin2
            dirp_pin = self.dirp_pin2
            dirm_pin = self.dirm_pin2
        else:
            raise ValueError("Invalid motor ID. Use 1 for Motor 1 or 2 for Motor 2.")

        # Set direction
        self.board.digital[dirp_pin].write(direction)
        self.board.digital[dirm_pin].write(not direction)

        # Pulse the motor
        for _ in range(total_pulses):
            self.board.digital[pul_pin].write(1)
            time.sleep(delay)
            self.board.digital[pul_pin].write(0)
            time.sleep(delay)

    def moveArmTo(self, coor):
        """
        Moves the robot arm to the specified coordinates.

        Parameters
        ----------
        coor : list
            A list of coordinates to move the arm to.

        Returns
        -------
        None
        """
        highcoor = coor[:2] + [400] + coor[3:]
        x = coor[0]
        y = coor[1]
        new_angle = math.atan2(y, x) / math.pi * 180
        new_angle += 180

        self.arm.set_servo_angle(servo_id=1, wait=True, angle=new_angle, is_radian=False, speed=100)
        self.arm.set_position_aa(highcoor, is_radian=False, speed=100, mvacc=100, wait=True)
        self.arm.set_position_aa(coor, is_radian=False, speed=80, mvacc=100, wait=True)

        self.arm.set_vacuum_gripper(on=False)
        self.arm.set_tcp_load(weight=0.61, center_of_gravity=(0.06125, 0.0458, 0.0375))
        self.arm.set_position_aa(highcoor, is_radian=False, speed=100, mvacc=100, wait=True)
        self.arm.set_servo_angle(angle=[180, 75, -180, 20, 0, 90, -60], speed=100, is_radian=False, wait=True)
        print("Arm moved to the specified coordinates.")

    
    def close(self):
        """
        Close the Arduino and xArm connections properly.

        Returns
        -------
        None
        """
        if self.ena_pin1 is not None:
            self.board.digital[self.ena_pin1].write(1)  # Disable the driver
        if self.ena_pin2 is not None:
            self.board.digital[self.ena_pin2].write(1)  # Disable the driver
        self.board


def main():
    # Initialize the RobotArm class with xArm IP and Arduino port
    arm = RobotArm(ip='192.168.1.241', arduino_port='COM4')  # Adjust IP and port as needed

    try:
        # Test rotating Motor 1
        print("Rotating Motor 1 in one direction (2 revolutions)...")
        arm.rotate_motor(motor_id=1, revolutions=2, direction=True, speed_rpm=60)  # Rotate Motor 1 clockwise
        print("Rotating Motor 1 in the opposite direction (2 revolutions)...")
        arm.rotate_motor(motor_id=1, revolutions=2, direction=False, speed_rpm=60)  # Rotate Motor 1 counter-clockwise
        
        # Test rotating Motor 2
        print("Rotating Motor 2 in one direction (1 revolution)...")
        arm.rotate_motor(motor_id=2, revolutions=1, direction=True, speed_rpm=30)  # Rotate Motor 2 clockwise
        print("Rotating Motor 2 in the opposite direction (1 revolution)...")
        arm.rotate_motor(motor_id=2, revolutions=1, direction=False, speed_rpm=30)  # Rotate Motor 2 counter-clockwise

    except KeyboardInterrupt:
        print("Operation interrupted by user.")

    finally:
        # Close connections and reset pins if needed
        arm.close()

if __name__ == '__main__':
    main()