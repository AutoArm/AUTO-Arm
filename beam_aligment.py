#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
# Notice
#   1. Changes to this file on Studio will not be preserved
#   2. The next conversion will overwrite the file with the same name
# 
# xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK
#   1. git clone git@github.com:xArm-Developer/xArm-Python-SDK.git
#   2. cd xArm-Python-SDK
#   3. python setup.py install
"""
import sys
import math
import time
import queue
import datetime
import random
import traceback
import threading
from xarm.wrapper import XArmAPI
import cv2
import numpy as np
from xarm.wrapper import XArmAPI
import os
import sys
import time
import math
import cv2
import pyrealsense2 as rs
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from PIL import Image
from xarm.wrapper import XArmAPI
if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../robot.conf')
        ip = parser.get('xArm', 'ip')
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit')
            sys.exit(1)
def hangle_err_warn_changed(item):
    print('ErrorCode: {}, WarnCode: {}'.format(item['error_code'], item['warn_code']))

arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
print("init done")

def extract_euler_angles(rvec):
    # Convert rotation vector to rotation matrix
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    
    # Extract Euler angles (Pitch, Yaw, Roll) from rotation matrix
    sy = np.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2)
    singular = sy < 1e-6
    
    if not singular:
        x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        y = np.arctan2(-rotation_matrix[2, 0], sy)
        z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        x = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        y = np.arctan2(-rotation_matrix[2, 0], sy)
        z = 0
    
    # Convert angles from radians to degrees
    pitch = x * 180 / np.pi
    yaw = y * 180 / np.pi
    roll = z * 180 / np.pi
    
    return pitch, yaw, roll
def extract_pitch_from_rotation(rvec):
    # Convert the rotation vector to a rotation matrix
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    
    # Extract pitch angle from the rotation matrix
    # Pitch angle is the rotation around the x-axis
    pitch = np.arctan2(rotation_matrix[2][1], rotation_matrix[2][2])
    
    # Convert pitch angle from radians to degrees
    pitch_degrees = pitch * 180 / np.pi
    
    return pitch_degrees

def detect_aruco(image, target_id):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is not None and len(ids) > 0:
        ids = ids.flatten()
        if target_id in ids:
            index = np.where(ids == target_id)[0][0]
            return corners[index][0], int(ids[index])
    return None, None

def estimate_pose(corners, mtx, dist, marker_length):
    # Define the 3D coordinates of the marker corners in the marker's coordinate system
    obj_points = np.array([
        [-marker_length / 2, marker_length / 2, 0],
        [marker_length / 2, marker_length / 2, 0],
        [marker_length / 2, -marker_length / 2, 0],
        [-marker_length / 2, -marker_length / 2, 0]
    ])

    # Solve for pose
    success, rvec, tvec = cv2.solvePnP(obj_points, corners, mtx, dist)
    return rvec, tvec

def draw_axis(img, rvec, tvec, mtx, dist):
    cv2.drawFrameAxes(img, mtx, dist, rvec, tvec, 0.1)  # length of the axis is 0.1 meter

def average_rotation_vectors(rvecs):
    rot_mats = [cv2.Rodrigues(rvec)[0] for rvec in rvecs]
    avg_rot_mat = np.mean(rot_mats, axis=0)
    U, _, Vt = np.linalg.svd(avg_rot_mat)
    avg_rot_mat = np.dot(U, Vt)
    avg_rvec, _ = cv2.Rodrigues(avg_rot_mat)
    return avg_rvec

    # Robot Main Run
def compute_transformation_matrix(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.hstack((R, tvec))
    T = np.vstack((T, [0, 0, 0, 1]))  # Add the homogeneous coordinate
    return T

def start():
    try:
        # Install xArm Gripper
        print("hither")
        code = arm.set_counter_reset()
        print("trying")
        weight = 0.610 
        center_of_gravity = (0.06125, 0.0458, 0.0375) 
        arm.set_tcp_load(weight=weight, center_of_gravity=center_of_gravity)
        code=arm.set_servo_angle(angle=[180,75,-180,20,0,90,-60],is_radian=False,speed=30,wait=True)

    except Exception as e:
        print('MainException: {}'.format(e))

def pickup(arm,coor,pipeline):
    print("Vacuum pickup at coordinate: ",coor)
    speeds=80
    x=coor[0]
    y=coor[1]
    new_angle=math.atan2(y,x)/math.pi*180
    new_angle+=180
    quad=0
    if x>=0 and y>=0:
        quad=1
    elif x<0 and y>=0:
        quad=2
        new_angle=280
    elif x<0 and y<0:
        quad=3
    else:
        quad=4
    highcoor=[coor[0]-75]+[coor[1]-35]+[400]+coor[3:]
    code = arm.set_servo_angle(servo_id=1,angle=new_angle,wait=True,is_radian=False,speed=speeds)
    code = arm.set_position_aa(highcoor, speed=speeds,mvacc=100, wait=True)
    # code = arm.set_servo_angle(servo_id=7,angle=pitch,wait=True,is_radian=False,speed=speeds)
    height=fine_adjust(arm,pipeline)
    height=height-100
    height=400-height
    height=height
    print(height)
    code,place=arm.get_position_aa(is_radian=False)
    arm.set_vacuum_gripper(on=True)
    code = arm.set_position_aa(place[:2]+[height]+place[3:], speed=70,mvacc=100, wait=True)
    arm.set_tcp_load(weight=0.8, center_of_gravity=(0.06125, 0.0458, 0.0375))    
    code = arm.set_position_aa(place[:2]+[400]+place[3:], speed=speeds,mvacc=100, wait=True)
    code = arm.set_servo_angle(angle=[180,75,-180,20,0,90,-60],is_radian=False,speed=speeds)
    return height

def move_to(arm,coor):
    arm.set_gripper_enable(True)
    code = arm.set_gripper_speed(2000)
    print("Move to coordinate: ",coor)
    speeds=80
    x=coor[0]
    y=coor[1]

    new_angle=math.atan2(y,x)/math.pi*180
    new_angle+=180
    quad=0
    if x>=0 and y>=0:
        quad=1
    elif x<0 and y>=0:
        quad=2
        new_angle=280
    elif x<0 and y<0:
        quad=3
    else:
        quad=4
    height=coor[2]
    highcoor=[coor[0]]+[coor[1]]+[400]+coor[3:]
    code = arm.set_servo_angle(servo_id=1,angle=new_angle,wait=True,is_radian=False,speed=speeds)
    code = arm.set_position_aa(highcoor, speed=speeds,mvacc=100, wait=True)
    code = arm.set_position_aa(coor, speed=speeds,mvacc=100, wait=True)
    


def pickup_claw(arm,coor,pipeline):
    arm.set_gripper_enable(True)
    code = arm.set_gripper_speed(2000)
    arm.set_gripper_position(850,wait=True)
    print("Claw pickup at coordinate: ",coor)
    speeds=80
    x=coor[0]
    y=coor[1]

    new_angle=math.atan2(y,x)/math.pi*180
    new_angle+=180
    quad=0
    if x>=0 and y>=0:
        quad=1
    elif x<0 and y>=0:
        quad=2
        new_angle=280
    elif x<0 and y<0:
        quad=3
    else:
        quad=4

    highcoor=[coor[0]-75]+[coor[1]-35]+[400]+coor[3:]
    code = arm.set_servo_angle(servo_id=1,angle=new_angle,wait=True,is_radian=False,speed=speeds)
    code = arm.set_position_aa(highcoor, speed=speeds,mvacc=100, wait=True)
    height=fine_adjust(arm,pipeline)
    height=height-100
    height=400-height
    height=height
    print(height)
    code,place=arm.get_position_aa(is_radian=False)
    code = arm.set_position_aa(place[:2]+[height]+place[3:], speed=speeds,mvacc=100, wait=True)
    arm.set_gripper_position(670,wait=True) 

    arm.set_tcp_load(weight=0.8, center_of_gravity=(0.06125, 0.0458, 0.0375))
    code = arm.set_position_aa(place[:2]+[400]+place[3:], speed=speeds,mvacc=100, wait=True)
    code = arm.set_servo_angle(angle=[180,75,-180,20,0,90,-60],is_radian=False,speed=speeds)

    return height

def pickup_claw_stay(arm,coor,pipeline):
    arm.set_gripper_enable(True)
    code = arm.set_gripper_speed(1000)
    arm.set_gripper_position(850,wait=True)
    print("Claw pickup at coordinate: ",coor)
    speeds=80
    x=coor[0]
    y=coor[1]

    new_angle=math.atan2(y,x)/math.pi*180
    new_angle+=180
    quad=0
    if x>=0 and y>=0:
        quad=1
    elif x<0 and y>=0:
        quad=2
        new_angle=280
    elif x<0 and y<0:
        quad=3
    else:
        quad=4

    highcoor=[coor[0]-75]+[coor[1]-35]+[400]+coor[3:]
    code = arm.set_servo_angle(servo_id=1,angle=new_angle,wait=True,is_radian=False,speed=speeds)
    code = arm.set_position_aa(highcoor, speed=speeds,mvacc=100, wait=True)
    height=fine_adjust(arm,pipeline)
    height=height-100
    height=400-height
    height=height
    print(height)
    code,place=arm.get_position_aa(is_radian=False)
    code = arm.set_position_aa(place[:2]+[height]+place[3:], speed=speeds,mvacc=100, wait=True)
    code,place=arm.get_position_aa(is_radian=False)
    print(place)
    arm.set_gripper_position(670,wait=True) 

    arm.set_tcp_load(weight=0.8, center_of_gravity=(0.06125, 0.0458, 0.0375))
    

    return place
def drop(arm,coor):
    highcoor=coor[:2]+[400]+coor[3:]
    x=coor[0]
    y=coor[1]
    new_angle=math.atan2(y,x)/math.pi*180
    new_angle+=180

    if x>=0 and y>=0:
        quad=1
    elif x<0 and y>=0:
        quad=2
        new_angle=280
    elif x<0 and y<0:
        quad=3
    else:
        quad=4
    code=arm.set_servo_angle(servo_id=1,wait=True,angle=new_angle,is_radian=False,speed=100)
    code = arm.set_position_aa(highcoor,is_radian=False, speed=100,  mvacc=100, wait=True)
    code = arm.set_position_aa(coor,is_radian=False, speed=80,  mvacc=100, wait=True)

    arm.set_vacuum_gripper(on=False)
    arm.set_tcp_load(weight=0.61, center_of_gravity=(0.06125, 0.0458, 0.0375))
    code = arm.set_position_aa(highcoor,is_radian=False, speed=100,  mvacc=100, wait=True)
    code=arm.set_servo_angle(angle=[180,75,-180,20,0,90,-60],speed=100,is_radian=False,wait=True)
    return
def drop_claw(arm,coor):
    code = arm.set_gripper_speed(1000)

    highcoor=coor[:2]+[400]+coor[3:]
    endcoor=coor[:2]+[500]+coor[3:]
    x=coor[0]
    y=coor[1]
    new_angle=math.atan2(y,x)/math.pi*180
    new_angle+=180

    if x>=0 and y>=0:
        quad=1
    elif x<0 and y>=0:
        quad=2
        new_angle=280
    elif x<0 and y<0:
        quad=3
    else:
        quad=4
    code=arm.set_servo_angle(servo_id=1,wait=True,angle=new_angle,is_radian=False,speed=100)
    code = arm.set_position_aa(highcoor,is_radian=False, speed=100,  mvacc=100, wait=True)
    code = arm.set_position_aa(coor,is_radian=False, speed=80,  mvacc=100, wait=True)

    arm.set_gripper_position(800,wait=True)
    arm.set_tcp_load(weight=0.61, center_of_gravity=(0.06125, 0.0458, 0.0375))
    code = arm.set_position_aa(endcoor,is_radian=False, speed=100,  mvacc=100, wait=True)
    code=arm.set_servo_angle(angle=[180,75,-180,20,0,90,-60],speed=100,is_radian=False,wait=True)
    return
def validate(self,coor):
    x=coor[0]
    y=coor[1]
    code,val=self._arm.get_position_aa(is_radian=False)
    x_curr=val[0]
    y_curr=val[1]
    quad=0
    quadcurr=0
    if x>=0 and y>=0:
        quad=1
    elif x<=0 and y>=0:
        quad=2
    elif x<=0 and y<=0:
        quad=3
    else:
        quad=4

    if x_curr>=0 and y_curr>=0:
        quad_curr=1
    elif x_curr<=0 and y_curr>=0:
        quad_curr=2
    elif x_curr<=0 and y_curr<=0:
        quad_curr=3
    else:
        quad_curr=4

    code,angle=self._arm.get_servo_angle(servo_id=1,is_radian=False)
    print("angle",angle)
    new_angle=angle+(quad-quadcurr)*90
    new_angle=new_angle%360
    print("new_angle",new_angle)
    code=self._arm.set_servo_angle(servo_id=1,wait=True,angle=new_angle,is_radian=False)

    code = self._arm.set_position_aa(coor,is_radian=False, speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
    if not self._check_code(code, 'set_position'):
        return
    return
def average_pose(rvec1, tvec1, rvec2, tvec2):
        # Average the translation vectors
        tvec_avg = (tvec1 + tvec2) / 2

        # Convert rotation vectors to matrices
        R1, _ = cv2.Rodrigues(rvec1)
        R2, _ = cv2.Rodrigues(rvec2)

        # Average the rotation matrices
        R_avg = (R1 + R2) / 2

        # Ensure R_avg is a valid rotation matrix by orthogonalizing it using SVD
        U, _, Vt = np.linalg.svd(R_avg)
        R_avg_orthogonal = U @ Vt

        # Convert the averaged rotation matrix back to a rotation vector
        rvec_avg, _ = cv2.Rodrigues(R_avg_orthogonal)

        return rvec_avg, tvec_avg
def calculate_rotation_angle(corner):
    # Calculate the center of the marker
    center_x = (corner[0][0] + corner[1][0] + corner[2][0] + corner[3][0]) / 4
    center_y = (corner[0][1] + corner[1][1] + corner[2][1] + corner[3][1]) / 4

    # Calculate the midpoint of one of the sides (between corner[0] and corner[1])
    midpoint_x = (corner[0][0] + corner[1][0]) / 2
    midpoint_y = (corner[0][1] + corner[1][1]) / 2

    # Calculate the vector from the center to the midpoint
    vector_x = midpoint_x - center_x
    vector_y = midpoint_y - center_y

    # Calculate the angle in radians and then convert to degrees
    angle_rad = np.arctan2(vector_y, vector_x)
    angle_deg = np.degrees(angle_rad)

    # Ensure the angle is in the range 0 to 360 degrees

    return angle_deg

def fine_adjust(arm,pipeline):
    leave=False
    depth_image=None
    corners=None
    while not leave:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        corners, id = detect_aruco(color_image, target_id)
        print(corners)
        if id is not None:
            # Calculate the center of the marker
            center_x = int((corners[0][0] + corners[1][0] + corners[2][0] + corners[3][0]) / 4)
            center_y = int((corners[0][1] + corners[1][1] + corners[2][1] + corners[3][1]) / 4)
            
            print(f"Tag ID: {target_id} - Center (x, y): ({center_x}, {center_y})")
            movey=(320-center_x)/10
            movex=(240-center_y)/10
            if movex==0 and movey==0:
                leave=True
            code,place=arm.get_position_aa(is_radian=False)
            code = arm.set_position_aa([place[0]+movex]+[place[1]+movey]+place[2:], speed=50,mvacc=100, wait=True)
            
            # color_image_with_markers = cv2.aruco.drawDetectedMarkers(color_image, corners, target_id)
            # Draw center point
            cv2.circle(color_image, (center_x, center_y), 5, (0, 255, 0), -1)
            
            cv2.imshow('Detected ArUco Markers', color_image)
    depth_value = depth_image[center_y, center_x]
    rotation_angle = calculate_rotation_angle(corners)
    rotation_angle+=90
    code = arm.set_position_aa([place[0]+71]+[place[1]+37]+place[2:], speed=50,mvacc=100, wait=True)
    code,pos = arm.get_servo_angle(servo_id=7,is_radian=False)
    code = arm.set_servo_angle(servo_id=7,wait=True,angle=pos+rotation_angle,is_radian=False)


    print("depth",depth_value)
    return depth_value

def rgb_to_intensity_and_peak(frame):
    # Read the image
    
    # Convert RGB to grayscale (intensity)
    intensity = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    print(intensity.shape)
    # Find the position of the maximum intensity
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(intensity)

    return intensity, max_loc
def center_arm(arm,frame):
    # check_saturation(frame)
        
    intensity,laser_center = rgb_to_intensity_and_peak(frame)
    print(laser_center)
    center_z=laser_center[1]
    center_x=laser_center[0]
    height, width, channels = frame.shape  

    movex=(width//2-center_x)/10
    movez=(height//2-center_z)/10
    print("center found",center_x,center_z,480,640,"and",movex,movez)
    if laser_center is not None:
        cv2.circle(frame, laser_center, 10, (0, 255, 0), -1)
        cv2.waitKey(100)
        height, width, channels = frame.shape  
        movex=(width//2-center_x)/20
        movez=(height//2-center_z)/20
        if abs(movez)<=1 and abs(movex)<=1:
            return 1
        code,place=arm.get_position_aa(is_radian=False)
        target_move=[place[0]+movex/90]+[place[1]]+[place[2]-movez/90]+place[3:]
        if target_move[0]>400 or target_move[2]>320 or target_move[0]<200 or target_move[2]<190:
            return -1
        code = arm.set_position_aa(target_move, speed=50,mvacc=100, wait=True)
        return 0
    else:
        return -1
if __name__ == '__main__':
    print(cv2.__version__)
    # Load calibration dataS
    camera_coor=[343.797485, 386.645844, 264.287628, -129.934337, 124.828596, 0.956668]

    calibration_data = np.load('stereo_calibration2.npz')
    mtx1 = calibration_data['mtx1']
    dist1 = calibration_data['dist1']
    mtx2 = calibration_data['mtx2']
    dist2 = calibration_data['dist2']
    R = calibration_data['R']
    T = calibration_data['T']
    actual_distance = 0.23  # 23 cm
    calibrated_distance = 0.22  # 22 cm

    # Realsense Pipelines
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    pipeline.start(config)

    # Calculate the scaling factor
    scale_factor = actual_distance / calibrated_distance
    print(f"Scaling factor: {scale_factor}")

    # Adjust the translation vector
    T = T * scale_factor
    print(f"Scaled translation vector:\n{T}")

    # Compute projection matrices
    proj1 = mtx1 @ np.hstack((np.eye(3), np.zeros((3, 1))))
    proj2 = mtx2 @ np.hstack((R, T))

    # Define the ArUco dictionary and parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()

    # Main loop for real-time detection
    cap1 = cv2.VideoCapture(0, cv2.CAP_MSMF)
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
    cap2 = cv2.VideoCapture(4, cv2.CAP_MSMF)
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)

    target_id = 4
    arm.set_gripper_enable(True)
    code = arm.set_gripper_speed(2000)
    arm.set_gripper_position(850,wait=True)
    marker_length = 0.062  # Length of the marker's side in meters
    start()
    print(mtx1)
    print(mtx2)
    while True: 
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()

        if not ret1 or not ret2:
            print("Error: Could not read frame from one or both cameras.")
            break

        corners1, id1 = detect_aruco(frame1, target_id)
        corners2, id2 = detect_aruco(frame2, target_id)

        if corners1 is not None and corners2 is not None:
            rvec1, tvec1 = estimate_pose(corners1, mtx1, dist1, marker_length)
            rvec2, tvec2 = estimate_pose(corners2, mtx2, dist2, marker_length)

            print(f"Target ArUco tag detected!")
            print(f"Camera 1 - Rotation Vector:\n{rvec1}\nTranslation Vector:\n{tvec1}")
            print(f"Camera 2 - Rotation Vector:\n{rvec2}\nTranslation Vector:\n{tvec2}")

            avg_rvec = average_rotation_vectors([rvec1, rvec2])
            avg_tvec = np.mean([tvec1, tvec2], axis=0)

            print(f"Averaged Rotation Vector:\n{avg_rvec}\nAveraged Translation Vector:\n{avg_tvec}")

            draw_axis(frame1, avg_rvec, avg_tvec, mtx1, dist1)

            avg_rvec_text = f"Avg Rvec: {avg_rvec[0][0]:.2f}, {avg_rvec[1][0]:.2f}, {avg_rvec[2][0]:.2f}"
            avg_tvec_text = f"Avg Tvec: {avg_tvec[0][0]:.2f}, {avg_tvec[1][0]:.2f}, {avg_tvec[2][0]:.2f}"
            
            mem=avg_tvec.flatten()
            rot=avg_rvec.flatten()
            mem=mem.tolist()
            mem.extend([-180,0,0])
            mem[0]=-mem[0]*1000
            mem[1]=mem[1]*1000
            mem[2]=(1.2-mem[2])*1000
            pickup_claw(arm,mem,pipeline)
            move_to(arm,camera_coor)
            
            break
        

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    cap1.release()
    cap2.release()
    cap2 = cv2.VideoCapture(2)
    centered=False
    centers=[]
    save_folder = 'test_v3'
    if not os.path.exists(save_folder):
        os.makedirs(save_folder)

    # Initialize a counter for frame numbering
    frame_count = 0

    def save_frame(frame):
        global frame_count

        # Create the filename with frame number
        filename = os.path.join(save_folder, f'frame_{frame_count:04d}.png')  # Saves as frame_0000.png, frame_0001.png, ...

        # Save the current frame to the folder
        cv2.imwrite(filename, frame)
        cv2.imshow('Camera 1',frame)
        cv2.waitKey(1000)
        # Increment the frame counter
        frame_count += 1

    for step in range(14):
        leave=False

        while leave is False:
            ret, frame = cap2.read()
            if not ret:
                print("Error: Could not read frame.")
                break
            centered=center_arm(arm,frame)
            if centered==-1:
                break
            elif centered==1:
                save_frame(frame)
                leave=True
        if leave:
            code,place=arm.get_position_aa(is_radian=False)
            centers.append(place)
        else:
            break
        code,place=arm.get_position_aa(is_radian=False)
        code = arm.set_position_aa([place[0]]+[place[1]-step*6]+place[2:], speed=50,mvacc=100, wait=True)
    print(centers)
    coordinates_array = np.array(centers)

    # Separate the x and y coordinates
    # x_coords = coordinates_array[:, 0]
    # y_coords = coordinates_array[:, 1]

    # # Perform linear regression to find the slope (m) and intercept (b)
    # slope, intercept = np.polyfit(x_coords, y_coords, 1)
    # print("Slope: ",slope," Intercept: ",intercept)
    cv2.destroyAllWindows()

    pipeline.stop()


    (0.1848432539070972, 340.6015844147822) #prediction full
    (-0.16123984572221328, 544.4053214833644) # prediction dark
    [[347.950958, 263.198761, 262.303833, 140.780772, -112.16388, -0.160887], [347.950958, 263.198761, 262.303833, 140.780772, -112.16388, -0.160887], [347.950989, 257.198822, 262.303894, 140.779283, -112.16577, -0.155673], [348.26947, 245.198456, 262.053955, 140.78358, -112.160327, -0.173033], [348.764954, 227.198013, 261.751129, 140.789424, -112.152993, -0.195665], [349.399231, 203.197586, 261.383728, 140.795555, -112.145201, -0.216979], [349.943146, 173.197327, 260.902435, 140.800998, -112.138383, -0.236001], [350.788147, 137.197281, 260.339355, 140.807071, -112.130705, -0.253419], [351.768768, 95.197212, 259.746704, 140.812915, -112.123314, -0.267514], [352.987213, 47.197174, 259.161865, 140.818645, -112.116095, -0.277827], [354.440582, -6.802771, 258.538055, 140.824775, -112.108417, -0.285276], [356.017273, -66.802574, 257.886261, 140.830906, -112.100682, -0.291005], [357.813568, -132.801498, 257.320099, 140.837381, -112.092546, -0.293183], [359.680359, -204.801514, 256.978363, 140.843167, -112.085327, -0.288427]]
    #data from beam alignment