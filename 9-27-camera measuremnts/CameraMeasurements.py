import os
import sys
import pyrealsense2 as rs
import cv2
import numpy as np
from MeasurementTool import MeasurementTool
from RoboticXArm import RobotArm
from xarm.wrapper import XArmAPI
from Processing import Processing

class CameraMeasurements(MeasurementTool):
    def __init__(self):
        """
        Constructor for CameraMeasurements class.

        Initializes the camera pipeline and sets up the projection matrices,
        ArUco dictionary and parameters, and captures the video streams from both cameras.
        """
        print(cv2.__version__)
        # Load calibration data
        self.calibration_data = np.load('stereo_calibration2.npz')
        self.mtx1 = self.calibration_data['mtx1']
        self.dist1 = self.calibration_data['dist1']
        self.mtx2 = self.calibration_data['mtx2']
        self.dist2 = self.calibration_data['dist2']
        self.R = self.calibration_data['R']
        self.T = self.calibration_data['T']
        self.actual_distance = 0.23  # 23 cm
        self.calibrated_distance = 0.22  # 22 cm
        self.marker_length = 0.062  # Define marker length if used elsewhere

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        self.pipeline.start(config)

        # Calculate the scaling factor
        scale_factor = self.actual_distance / self.calibrated_distance

        # Adjust the translation vector
        self.T = self.T * scale_factor

        # Compute projection matrices
        self.proj1 = self.mtx1 @ np.hstack((np.eye(3), np.zeros((3, 1))))
        self.proj2 = self.mtx2 @ np.hstack((self.R, self.T))

        # Define the ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()

        # Main loop for real-time detection
        self.cap1 = cv2.VideoCapture(0, cv2.CAP_MSMF)
        self.cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
        self.cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
        self.cap2 = cv2.VideoCapture(1, cv2.CAP_MSMF)
        self.cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
        self.cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)

        print("cameras online")

    def see_camera(camera_id):
        cap = cv2.VideoCapture(camera_id, cv2.CAP_MSMF) # 0 and 1 are top cameras, 3 is the one grabbed
        if not cap.isOpened(): 
            print("Cannot open camera") 
            exit()
        while True: 
            ret, frame = cap.read() 
            if not ret: 
                print("Can't receive frame (stream end?). Exiting ...") 
                break 
            cv2.imshow('Camera Test', frame) 
            if cv2.waitKey(1) == ord('q'): 
                break
            
        cap.release()
        cv2.destroyAllWindows()

    def __del__(self):
        """
        Release all resources used by the CameraMeasurements instance.

        This is called automatically when the object is garbage collected.
        """
        self.pipeline.stop()
        self.cap1.release()
        self.cap2.release()
        cv2.destroyAllWindows()

    def detect_aruco(self, image, target_id):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is not None and len(ids) > 0:
            ids = ids.flatten()
            if target_id in ids:
                index = np.where(ids == target_id)[0][0]
                return corners[index][0], int(ids[index])
        return None, None

    def find_pose(self, target_id):
        ret1, frame1 = self.cap1.read()
        ret2, frame2 = self.cap2.read()
        if not ret1 or not ret2:
            raise RuntimeError("Failed to capture frames from both cameras.")
        
        # Convert frames to grayscale
        gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

        # Display the grayscale images
        cv2.imshow('Camera 1 - Grayscale', gray1)
        cv2.imshow('Camera 2 - Grayscale', gray2)

        # Wait for a short period to update the display
        cv2.waitKey(1)

        corners1, id1 = self.detect_aruco(frame1, target_id)
        corners2, id2 = self.detect_aruco(frame2, target_id)

        if corners1 is None or corners2 is None:
            raise ValueError("ArUco marker not detected in one or both frames.")

        rvec1, tvec1 = Processing.estimate_pose(corners1, self.mtx1, self.dist1, self.marker_length)
        rvec2, tvec2 = Processing.estimate_pose(corners2, self.mtx2, self.dist2, self.marker_length)
        avg_rvec = Processing.average_rotation_vectors([rvec1, rvec2])
        avg_tvec = np.mean([tvec1, tvec2], axis=0)

        return avg_rvec, avg_tvec


    # Placeholder methods to be implemented
    def measureBeamCenter(self, coorFrom, coorTo, steps):
        self.moveArmTo(coorFrom)

        pass

    def validatePosition(self, tvec):
        pass

    def measureBeamWidth(self, tvec):
        pass

def handle_err_warn_changed(item):
    print('ErrorCode: {}, WarnCode: {}'.format(item['error_code'], item['warn_code']))

def setup_xarm(ip=None):
    """
    Set up the xArm by initializing it with the correct IP address.
    If an IP address is not provided, it will attempt to retrieve it from
    command line arguments, a configuration file, or prompt the user.

    :param ip: Optional; the IP address of the xArm.
    :return: The initialized xArmAPI object.
    """
    if not ip:
        # Check if IP is provided via command line arguments
        if len(sys.argv) >= 2:
            ip = sys.argv[1]
        else:
            # Attempt to get IP from configuration file
            try:
                from configparser import ConfigParser
                parser = ConfigParser()
                parser.read('../robot.conf')
                ip = parser.get('xArm', 'ip')
            except Exception as e:
                print(f"Error reading IP from config file: {e}")
                ip = input('Please input the xArm IP address: ')
                if not ip:
                    print('Input error, exiting.')
                    sys.exit(1)

    # Initialize the xArm with the given IP address
    arm = XArmAPI(ip)
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    arm.register_error_warn_changed_callback(handle_err_warn_changed)
    print("xArm initialization done.")
    return arm

def start(arm):
    """
    Additional setup for the xArm, including gripper installation and setting TCP load.
    """
    try:
        # Install xArm Gripper
        print("Initializing additional settings for the xArm...")
        code = arm.set_counter_reset()
        print("Counter reset code:", code)
        weight = 0.610 
        center_of_gravity = (0.06125, 0.0458, 0.0375) 
        arm.set_tcp_load(weight=weight, center_of_gravity=center_of_gravity)
        code = arm.set_servo_angle(angle=[180, 75, -180, 20, 0, 90, -60], is_radian=False, speed=30, wait=True)
        print("xArm setup completed successfully.")

    except Exception as e:
        print('MainException:', e)

def main():
    """
    Main method to test the CameraMeasurements class and set up the robotic arm.
    """
    # Initialize the RobotArm object
    arm = RobotArm("192.168.1.241")  # You can pass an IP if needed, or let it handle the setup process

    # Set the target ArUco marker ID
    target_id = 4  

    # Initialize the CameraMeasurements object
    camera_measurements = CameraMeasurements()
    camera_measurements.see_camera(3)

    try:
        while True:
            
            # Find the pose of the target marker and display grayscale images
            avg_rvec, avg_tvec = camera_measurements.find_pose(target_id)
            print("Averaged Rotation Vector:\n", avg_rvec)
            print("Averaged Translation Vector:\n", avg_tvec)

            # Example use of the RobotArm class
            mem = avg_tvec.flatten().tolist()
            mem.extend([-180, 0, 0])
            mem[0] = -mem[0] * 1000
            mem[1] = mem[1] * 1000
            mem[2] = (1.2 - mem[2]) * 1000

            # Move the arm to the detected position
            arm.moveArmTo(mem)

            # Press 'q' to exit the loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    main()
