#!/usr/bin/env python3
import os
import time
import cv2
import traceback
import numpy as np
from xarm import version
from xarm.wrapper import XArmAPI
from matplotlib import pyplot as plt

def save_frame_as_numpy(save_folder, frame, frame_count, theta, phi):
        # Create the filename with frame number
        filename = os.path.join(save_folder, f'image_{frame_count:04d}.npy')  # Saves as frame_0000.npy, frame_0001.npy, ...
        # Save the current frame as a NumPy array to a file
        np.savez(filename, frame=frame, theta=theta, phi=phi)
        #np.save(filename, frame)


def capture_averaged_image(cap, n_avg):
    for indx in range(n_avg):
        ret, frame = cap.read()
        if not ret:
            return None
        if indx==0:
            frames=frame.astype(np.float32)
        else:
            frames = frames + frame.astype(np.float16)
    

    avg_frame = frames/n_avg
    return avg_frame

def spherical_to_rpy(theta_deg, phi_deg):
    """
    Convert spherical angles (theta, phi) to simple xArm R-P-Y (deg).
    :param theta_deg: azimuth angle about the base Z-axis (0-360)
    :param phi_deg:   polar angle from +Z downward (0-180)
    :return: (roll, pitch, yaw) in degrees
    """
    yaw = theta_deg
    pitch = phi_deg - 90.0
    roll = 0.0
    return (roll, pitch, yaw)

class RobotMain(object):
    def __init__(self, robot):
        self.alive = True
        self._arm = robot
        self._ignore_exit_state = False

        # Speeds and accelerations
        self._tcp_speed = 100   # mm/s for linear moves
        self._tcp_acc   = 2000  # mm/s^2 for linear moves
        self._angle_speed = 20  # deg/s for joint moves
        self._angle_acc = 500   # deg/s^2 for joint moves

        self._robot_init()

    def _robot_init(self):
        """Prepare the robot for motion."""
        # Clear errors/warnings
        self._arm.clean_warn()
        self._arm.clean_error()

        # Enable motion, set mode/state
        self._arm.motion_enable(True)
        self._arm.set_mode(0)   # 0: Position (joint/linear) mode
        self._arm.set_state(0)  # 0: Ready state
        time.sleep(1)

        # Register callbacks to monitor errors/states
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)

        # -------------------------------------------------------------------
        # 1) Set a TCP (tool) offset so that the "tip" or lens center is
        #    the origin of the tool frame.
        #    Adjust [x, y, z, roll, pitch, yaw] as needed.
        # -------------------------------------------------------------------
        offset = [0.0, 0.0, 185, 0.0, 0.0, 0.0]  # Example: 100 mm along +Z
        code = self._arm.set_tcp_offset(offset)
        if code != 0:
            self.pprint(f"Failed to set tool offset, code={code}")
        else:
            self.pprint(f"Tool offset set: {offset}")

        # Give the controller a moment to process the new offset
        time.sleep(1)

        # Re-initialize mode/state after offset set (important)
        self._arm.motion_enable(True)
        self._arm.set_mode(0)   # position mode
        self._arm.set_state(0)  # ready
        time.sleep(1)

    def _error_warn_changed_callback(self, data):
        """Handle error/warning from xArm."""
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint(f"Error code = {data['error_code']}, shutting down.")
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    def _state_changed_callback(self, data):
        """Stop if the arm state becomes 4 (Emergency stop) unless ignoring."""
        if not self._ignore_exit_state and data and data['state'] == 4:
            self.alive = False
            self.pprint('State=4 (Emergency stop), shutting down.')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    @staticmethod
    def pprint(*args, **kwargs):
        """Print with a timestamp and the calling line number."""
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][Line {}] {}'.format(
                time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())),
                stack_tuple.lineno,
                ' '.join(map(str, args))
            ))
        except:
            print(*args, **kwargs)

    @property
    def is_alive(self):
        """Check if robot is in a good state."""
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._ignore_exit_state:
                return True
            # If state == 5 (Paused), wait a bit to see if it recovers
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False

    def _check_code(self, code, label=""):
        """Helper to mark 'not alive' if code != 0."""
        if (not self.is_alive) or code != 0:
            self.alive = False
            self.pprint(
                f"Command '{label}' returned code={code}, "
                f"state={self._arm.state}, error={self._arm.error_code}"
            )
        return self.is_alive

    def run(self):



        # data=np.load('sample_transmission_theta_phi/image_0001.npy.npz')
        # plt.imshow(np.round(data['frame']).astype(int), interpolation='nearest')
        # plt.show()
        # cv2.waitKey(1000)

        # raise SystemExit

        cap = cv2.VideoCapture(1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 8000)  
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 6000) 

        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Camera resolution: {actual_width}x{actual_height}")

        save_folder = "sample_transmission_theta_phi"


        
        # plt.imshow(np.round(frame).astype(int), interpolation='nearest')
        # plt.show()
        # cv2.waitKey(1000)






        """
        Main routine:
          1) Move to a safe joint pose
          2) Keep tip at the same (x, y, z)
          3) Rotate in spherical angles (theta, phi)
        """

        frame_count = 1
        try:
            # 1) Move the arm to a safe joint pose
            #    If [0,0,0,0,0,0,0] is invalid,
            #    choose a known safe pose instead.



            # home_pose = [0, 0, 0, 0, 0, 0, 0] ## HOME POSITION
            # code = self._arm.set_servo_angle(
            #     angle=home_pose,
            #     speed=self._angle_speed,
            #     mvacc=self._angle_acc,
            #     wait=True
            # )
            # if not self._check_code(code, "move_home"):
            #     return

            # 2) Define the tip position we want to keep in space (base frame)
            #    For example, let's keep the tip at (x=300, y=0, z=200).
            x_fixed = -243.5 - 15
            y_fixed = 188.3 + 17.5
            z_fixed = 172.3 -4.5

            # 3) Spherical angles to try
            angles_to_try = [(0,0)]#, (0,30), (0,-30), (30, 0), (30, 0), (-30,0), (0,0)]

            # for x in range(-5, 41):
            #     for y in range(-5, 41):
            #         if x % 2 == 0:
            #             y = 35 - y
                    
            #         angles_to_try.append((x,y))



            # Speed/acc for linear moves
            speed_lin = 50    # mm/s
            mvacc_lin = 1000  # mm/s^2
            
            buffer = 1 
            for (theta_deg, phi_deg) in angles_to_try:

                theta_deg += 90
                phi_deg -= 90
                if not self.is_alive:
                    break

                # Convert (theta, phi) -> (roll, pitch, yaw)
                roll, pitch, yaw = spherical_to_rpy(theta_deg, phi_deg)

                # Move to the same x,y,z, but new orientation
                code = self._arm.set_position(
                    x=x_fixed, y=y_fixed, z=z_fixed,
                    roll=roll, pitch=pitch, yaw=yaw,
                    speed=speed_lin, mvacc=mvacc_lin,
                    wait=True
                )
                if not self._check_code(code, f"set_position(theta={theta_deg}, phi={phi_deg})"):
                    return
                

                time.sleep(1)
                self.pprint(
                    f"TIP at (x={x_fixed}, y={y_fixed}, z={z_fixed}) | "
                    f"Orientation => theta={theta_deg-90}, phi={phi_deg+90}"
                )

                if buffer == 1:
                    #time.sleep(60)
                    buffer = 0

                

                # frame = capture_averaged_image(cap, 64)
                # frame = frame[2350:3450,:,:]
                # save_frame_as_numpy(save_folder, frame, frame_count, theta_deg - 90, phi_deg + 90)
                # frame_count += 1 






                

            # Optionally, move back to home at the end
            # code = self._arm.set_servo_angle(
            #     angle=home_pose,
            #     speed=self._angle_speed,
            #     mvacc=self._angle_acc,
            #     wait=True
            # )
            # self._check_code(code, "return_home")

        except Exception as e:
            self.pprint(f"MainException: {e}")
        finally:
            self.alive = False
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
            self._arm.release_state_changed_callback(self._state_changed_callback)

if __name__ == '__main__':
    RobotMain.pprint(f"xArm-Python-SDK Version: {version.__version__}")
    # Update the IP address for your xArm 7
    arm = XArmAPI("192.168.1.241")
    robot_main = RobotMain(arm)
    robot_main.run()