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
import pyfirmata
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
import serial

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from PIL import Image
from xarm.wrapper import XArmAPI
# if len(sys.argv) >= 2:
#     ip = sys.argv[1]
# else:
#     try:
#         from configparser import ConfigParser
#         parser = ConfigParser()
#         parser.read('../robot.conf')
#         ip = parser.get('xArm', 'ip')
#     except:
#         ip = input('Please input the xArm ip address:')
#         if not ip:
#             print('input error, exit')
#             sys.exit(1)
# def hangle_err_warn_changed(item):
#     print('ErrorCode: {}, WarnCode: {}'.format(item['error_code'], item['warn_code']))
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
 
arm = XArmAPI("192.168.1.241")
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
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()
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
        # print("hither")
        code = arm.set_counter_reset()
        # print("trying")
        weight = 0.610 
        center_of_gravity = (0.06125, 0.0458, 0.0375) 
        arm.set_tcp_load(weight=weight, center_of_gravity=center_of_gravity)
        code=arm.set_servo_angle(angle=[180,75,-180,20,0,90,-60],is_radian=False,speed=30,wait=True)

    except Exception as e:
        print('MainException: {}'.format(e))
def move_to(arm,coor):
    # arm.set_gripper_enable(True)
    # code = arm.set_gripper_speed(2000)
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
    highcoor=[coor[0]]+[coor[1]]+[550]+coor[3:]
    code = arm.set_servo_angle(servo_id=1,angle=new_angle,wait=True,is_radian=False,speed=speeds)
    code = arm.set_position_aa(highcoor, speed=speeds,mvacc=100, wait=True)
    code = arm.set_position_aa(coor, speed=speeds,mvacc=100, wait=True)

def center_arm(arm,frame):
    # check_saturation(frame)
        
    intensity,laser_center = rgb_to_intensity_and_peak(frame)
    print(laser_center)
    center_z=laser_center[1]
    center_x=laser_center[0]
    height, width, channels = frame.shape  
    
    if laser_center is not None:
        cv2.circle(frame, laser_center, 10, (0, 255, 0), -1)
        cv2.imshow("cams",frame)
        cv2.waitKey(100)
        movex=(width//2-center_x)/110
        movez=(height//2-center_z)/110
        print("center found",center_x,center_z,width,height,"and move right ",movex,movez)
        if abs(movez)<=1 and abs(movex)<=1:
            return 1
        code,place=arm.get_position_aa(is_radian=False)
        target_move=[place[0]+movex/10]+[place[1]]+[place[2]-movez/10]+place[3:]
        if target_move[0]>400 or target_move[2]>320 or target_move[0]<200 or target_move[2]<190:
            return -1
        code = arm.set_position_aa(target_move, speed=20,mvacc=100, wait=True)
        return 0
    else:
        return -1   
def pickup_claw(arm,coor,pipeline,target_id,special=False):
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
    if coor[0]<-300:
        coor[0]+=10
    if coor[0]>300:
        coor[0]-=10
    highcoor=[coor[0]-70]+[coor[1]-35]+[400]+coor[3:]
    # print("here",highcoor)
    code = arm.set_servo_angle(servo_id=1,angle=new_angle,wait=True,is_radian=False,speed=speeds)
    code = arm.set_position_aa(highcoor, speed=speeds,mvacc=100, wait=True)
    height,rotation=fine_adjust(arm,pipeline,target_id)
    print("after fine adjust",height)
    height=height-100
    height=400-height
    if special:
        height+=20
    else:
        height+=16
    
    
    code,place=arm.get_position_aa(is_radian=False)
    pickup_pos=place[:2]+[height]+place[3:]
    code = arm.set_position_aa(place[:2]+[height]+place[3:], speed=speeds,mvacc=100, wait=True)
    if special:
        arm.set_gripper_position(300,wait=True) 
    else:
        arm.set_gripper_position(530,wait=True)

    arm.set_tcp_load(weight=0.8, center_of_gravity=(0.06125, 0.0458, 0.0375))
    code = arm.set_position_aa(place[:2]+[400]+place[3:], speed=speeds,mvacc=100, wait=True)
    code = arm.set_servo_angle(angle=[180,75,-180,20,0,90,-60],is_radian=False,speed=speeds)

    return pickup_pos,rotation
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
    height,rotation=fine_adjust(arm,pipeline)
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

def drop_claw(arm,coor):
    code = arm.set_gripper_speed(1000)
    print("coor",coor)
    highcoor=coor[:2]+[500]+coor[3:]
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
    mid_coor=coor[:2]+[coor[2]+1.5]+coor[3:]
    code=arm.set_servo_angle(servo_id=1,wait=True,angle=new_angle,is_radian=False,speed=50)
    code = arm.set_position_aa(highcoor,is_radian=False, speed=80,  mvacc=100, wait=True)
    code = arm.set_position_aa(mid_coor,is_radian=False, speed=80,  mvacc=100, wait=True)
    code = arm.set_position_aa(coor,is_radian=False, speed=20,  mvacc=100, wait=True)
    arm.set_gripper_position(850,wait=True)
    arm.set_tcp_load(weight=0.8, center_of_gravity=(0.06125, 0.0458, 0.0375))
    code = arm.set_gripper_speed(2000)
    code = arm.set_position_aa(endcoor,is_radian=False, speed=100,  mvacc=100, wait=True)
    code=arm.set_servo_angle(angle=[180,75,-180,20,0,90,-60],speed=80,is_radian=False,wait=True)
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
def calculate_area(corner):
    # Use the Shoelace formula
    x_coords = [corner[i][0] for i in range(4)]
    y_coords = [corner[i][1] for i in range(4)]
    
    # Calculate the area using the Shoelace formula
    area = 0.5 * abs(
        x_coords[0] * y_coords[1] +
        x_coords[1] * y_coords[2] +
        x_coords[2] * y_coords[3] +
        x_coords[3] * y_coords[0] -
        (y_coords[0] * x_coords[1] +
         y_coords[1] * x_coords[2] +
         y_coords[2] * x_coords[3] +
         y_coords[3] * x_coords[0])
    )
    
    return area
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
def fine_adjust(arm,pipeline,target_id):
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
        # print(corners, "corners")
        if id is not None:
            # Calculate the center of the marker
            center_x = int((corners[0][0] + corners[1][0] + corners[2][0] + corners[3][0]) / 4)
            center_y = int((corners[0][1] + corners[1][1] + corners[2][1] + corners[3][1]) / 4)
            
            # print(f"Tag ID: {target_id} - Center (x, y): ({center_x}, {center_y})")
            movey=(320-center_x)/10
            movex=(240-center_y)/10
            if movex==0 and movey==0:
                leave=True
            code,place=arm.get_position_aa(is_radian=False)
            code = arm.set_position_aa([place[0]+movex]+[place[1]+movey]+place[2:], speed=20,mvacc=30, wait=True)
            # print(movex,movey)
            # color_image_with_markers = cv2.aruco.drawDetectedMarkers(color_image, corners, target_id)
            # Draw center point
            # cv2.circle(color_image, (center_x, center_y), 5, (0, 255, 0), -1)
            
            # cv2.imshow('Detected ArUco Markers', color_image)
    depth_value = depth_image[center_y, center_x]
    h, w = depth_image.shape
    center_h = h // 2
    center_w = w // 2
   
    # Get 5x5 square centered at middle
    square = depth_image[center_h-1:center_h+2, center_w-1:center_w+2]
    depth_value=np.mean(square)
    print(depth_value,"depth")
    rotation_angle = calculate_rotation_angle(corners)
    rotation_angle+=90
    code = arm.set_position_aa([place[0]+72.2]+[place[1]+33.5]+place[2:], speed=50,mvacc=100, wait=True)
    code,pos = arm.get_servo_angle(servo_id=7,is_radian=False)
    code = arm.set_servo_angle(servo_id=7,wait=True,angle=pos+rotation_angle,is_radian=False)


    # print("depth",depth_value)
    return depth_value,rotation_angle
def fine_adjust_wo_move(arm,pipeline,target_id):
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
        # print(corners, "corners")
        if id is not None:
            # Calculate the center of the marker
            center_x = int((corners[0][0] + corners[1][0] + corners[2][0] + corners[3][0]) / 4)
            center_y = int((corners[0][1] + corners[1][1] + corners[2][1] + corners[3][1]) / 4)
            
            # print(f"Tag ID: {target_id} - Center (x, y): ({center_x}, {center_y})")
            movey=(320-center_x)/15
            movex=(240-center_y)/15
            if movex==0 and movey==0:
                leave=True
            code,place=arm.get_position_aa(is_radian=False)
            code = arm.set_position_aa([place[0]+movex]+[place[1]+movey]+place[2:], speed=20,mvacc=50, wait=True)
            
            # color_image_with_markers = cv2.aruco.drawDetectedMarkers(color_image, corners, target_id)
            # Draw center point
            cv2.circle(color_image, (center_x, center_y), 5, (0, 255, 0), -1)
            
            cv2.imshow('Detected ArUco Markers', color_image)
    depth_value = depth_image[center_y, center_x]
    area=abs(calculate_area(corners))
    return area
    return depth_value
def rgb_to_intensity_and_peak(frame):
    # Read the image
    
    # Convert RGB to grayscale (intensity)
    intensity = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    print(intensity.shape)

    # Find the position of the maximum intensity
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(intensity)

    return intensity, max_loc
def find_pos(arm,coor,pipeline,target_id):
    arm.set_gripper_enable(True)
    code = arm.set_gripper_speed(2000)
    arm.set_gripper_position(850,wait=True)
    # print("Claw pickup at coordinate: ",coor)
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

    highcoor=[coor[0]-70]+[coor[1]-35]+[350]+coor[3:]
    code = arm.set_servo_angle(servo_id=1,angle=new_angle,wait=True,is_radian=False,speed=speeds)
    code = arm.set_position_aa(highcoor, speed=speeds,mvacc=100, wait=True)
    height,rotation=fine_adjust(arm,pipeline,target_id)
    # print("after fine adjust")
    # height=height-100
    height=350-height
    height=height+100
    print(height,"height of object")
    code,place=arm.get_position_aa(is_radian=False)
    pickup_pos=place[:2]+[height]+place[3:]
    # code = arm.set_position_aa(place[:2]+[height]+place[3:], speed=speeds,mvacc=100, wait=True)
    

    # code = arm.set_position_aa(place[:2]+[400]+place[3:], speed=speeds,mvacc=100, wait=True)
    # code = arm.set_servo_angle(angle=[180,75,-180,20,0,90,-60],is_radian=False,speed=speeds)
    gohome()
    # print("Place found: ",place)
    return place,rotation
def rotate_claw(arm,rotation_angle):
    code,pos = arm.get_servo_angle(servo_id=7,is_radian=False)
    code = arm.set_servo_angle(servo_id=7,wait=True,angle=pos+rotation_angle,is_radian=False)
# Methods to be used
def rotate_motor_async(board):
    ccw_pin1 = 8  
    cw_pin1 = 9  
    stop_pin1 = 7  

    ccw_pin2 = 5  
    cw_pin2 = 6
    stop_pin2 = 4  
    # Set up the pins
    board.digital[ccw_pin1].mode = pyfirmata.OUTPUT
    board.digital[cw_pin1].mode = pyfirmata.OUTPUT
    board.digital[stop_pin1].mode = pyfirmata.OUTPUT
    board.digital[ccw_pin2].mode = pyfirmata.OUTPUT
    board.digital[cw_pin2].mode = pyfirmata.OUTPUT
    board.digital[stop_pin2].mode = pyfirmata.OUTPUT


        
    board.digital[cw_pin1].write(1)  # Enable the driver (active LOW)
    time.sleep(0.1)
    board.digital[cw_pin1].write(0)  # Enable the driver (active LOW)
    # print("cw 1 enabled")

    board.digital[cw_pin2].write(1)  # Enable the driver (active LOW)
    time.sleep(0.1)
    board.digital[cw_pin2].write(0)  # Enable the driver (active LOW)
    # print("cw 2 enabled")
def rotate_motor1(controller):
    controller.power_motor(0, True)
    controller.move_motor(1, 2048)
def rotate_motor(revolutions, direction, board,option=0,speed_rpm=60):
    pul_pin1,dirp_pin1,dirm_pin1,ena_pin1=4,3,2,None
    pul_pin2,dirp_pin2,dirm_pin2,ena_pin2=9,7,5,None
    """
    Rotate the motor a specified number of revolutions
    :param revolutions: Number of revolutions (can be fractional)
    :param direction: True for one direction, False for the opposite
    :param speed_rpm: Speed in rotations per minute
    """

    # Set up the pins
    board.digital[pul_pin1].mode = pyfirmata.OUTPUT
    board.digital[dirp_pin1].mode = pyfirmata.OUTPUT
    board.digital[dirm_pin1].mode = pyfirmata.OUTPUT

    board.digital[pul_pin2].mode = pyfirmata.OUTPUT
    board.digital[dirp_pin2].mode = pyfirmata.OUTPUT
    board.digital[dirm_pin2].mode = pyfirmata.OUTPUT

    if ena_pin1 is not None:
        board.digital[ena_pin2].mode = pyfirmata.OUTPUT
        board.digital[ena_pin2].write(0)  # Enable the driver (active LOW)

    if ena_pin1 is not None:
        board.digital[ena_pin1].mode = pyfirmata.OUTPUT
        board.digital[ena_pin1].write(0)  # Enable the driver (active LOW)

    # Motor and driver configuration
    steps_per_revolution = 200  # This is typically 200 for a 1.8° stepper motor
    microstep_division = 1  # Set this to your DM542T's microstep setting

    # Calculate total pulses per revolution
    pulses_per_revolution = steps_per_revolution * microstep_division

    total_pulses = int(abs(revolutions) * pulses_per_revolution)
    delay = (60 / (speed_rpm * pulses_per_revolution)) / 2

    # Set direction
    board.digital[dirp_pin1].write(direction)
    board.digital[dirm_pin1].write(not direction)

    board.digital[dirp_pin2].write(direction)
    board.digital[dirm_pin2].write(not direction)

    if option==0:
    # Pulse the motor
        for count in range(total_pulses):
            if count%2==0:
                board.digital[pul_pin1].write(1)
                time.sleep(delay)
                board.digital[pul_pin1].write(0)
                time.sleep(delay)
            else:
                board.digital[pul_pin2].write(1)
                time.sleep(delay)
                board.digital[pul_pin2].write(0)
                time.sleep(delay)
    elif option==1:
        for _ in range(total_pulses):
            board.digital[pul_pin1].write(1)
            time.sleep(delay)
            board.digital[pul_pin1].write(0)
            time.sleep(delay)
    else:
        for _ in range(total_pulses):
            board.digital[pul_pin2].write(1)
            time.sleep(delay)
            board.digital[pul_pin2].write(0)
            time.sleep(delay)
def initialize_cams():
    calibration_data = np.load('stereo_calibration2.npz')
    mtx1 = calibration_data['mtx1']
    dist1 = calibration_data['dist1']
    mtx2 = calibration_data['mtx2']
    dist2 = calibration_data['dist2']
    R = calibration_data['R']
    T = calibration_data['T']
    actual_distance = 0.23  # 23 cm
    calibrated_distance = 0.22  # 22 cm
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    pipeline.start(config)

    # Calculate the scaling factor
    scale_factor = actual_distance / calibrated_distance
    # print(f"Scaling factor: {scale_factor}")

    # Adjust the translation vector
    T = T * scale_factor
    # print(f"Scaled translation vector:\n{T}")

    # Compute projection matrices
    proj1 = mtx1 @ np.hstack((np.eye(3), np.zeros((3, 1))))
    proj2 = mtx2 @ np.hstack((R, T))
    cap1 = cv2.VideoCapture(0, cv2.CAP_MSMF)
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
    cap2 = cv2.VideoCapture(5, cv2.CAP_MSMF) #door
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
    cap3 = cv2.VideoCapture(2, cv2.CAP_MSMF)
    cap3.set(cv2.CAP_PROP_FRAME_WIDTH, 8000)
    cap3.set(cv2.CAP_PROP_FRAME_HEIGHT, 6000)
    print("cameras setup")
    return (cap1,cap2,cap3,pipeline)
def pickup_element_with_tag(cams,tag_id,size=0.062,special=False):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()
    cap1,cap2,cap3,pipeline=cams
    calibration_data = np.load('stereo_calibration2.npz')
    mtx1 = calibration_data['mtx1']
    dist1 = calibration_data['dist1']
    mtx2 = calibration_data['mtx2']
    dist2 = calibration_data['dist2']
    target_id = tag_id
    arm.set_gripper_enable(True)
    code = arm.set_gripper_speed(2000)
    arm.set_gripper_position(850,wait=True)
    marker_length = size  # Length of the marker's side in meters
    # print("inloop")
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

            # print(f"Target ArUco tag detected!")
            # print(f"Camera 1 - Rotation Vector:\n{rvec1}\nTranslation Vector:\n{tvec1}")
            # print(f"Camera 2 - Rotation Vector:\n{rvec2}\nTranslation Vector:\n{tvec2}")

            avg_rvec = average_rotation_vectors([rvec1, rvec2])
            avg_tvec = np.mean([tvec1, tvec2], axis=0)

            # print(f"Averaged Rotation Vector:\n{avg_rvec}\nAveraged Translation Vector:\n{avg_tvec}")
            avg_rvec_text = f"Avg Rvec: {avg_rvec[0][0]:.2f}, {avg_rvec[1][0]:.2f}, {avg_rvec[2][0]:.2f}"
            avg_tvec_text = f"Avg Tvec: {avg_tvec[0][0]:.2f}, {avg_tvec[1][0]:.2f}, {avg_tvec[2][0]:.2f}"
            
            mem=avg_tvec.flatten()
            rot=avg_rvec.flatten()
            mem=mem.tolist()
            mem.extend([-180,0,0])
            mem[0]=-mem[0]*1000
            mem[1]=mem[1]*1000
            mem[2]=(1.2-mem[2])*1000
            pos=None
            if special:
                pos,rotation=pickup_claw(arm,mem,pipeline,target_id,True)
                arm.set_tcp_load(weight=1.3, center_of_gravity=(0.06125, 0.0458, 0.0375))

            else:
                pos,rotation=pickup_claw(arm,mem,pipeline,target_id)
            
            return pos,rotation
        

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
def find_position_of_tag(cams,tag_id,size=0.062):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()
    cap1,cap2,cap3,pipeline=cams
    calibration_data = np.load('stereo_calibration2.npz')
    mtx1 = calibration_data['mtx1']
    dist1 = calibration_data['dist1']
    mtx2 = calibration_data['mtx2']
    dist2 = calibration_data['dist2']
    target_id = tag_id
    arm.set_gripper_enable(True)
    code = arm.set_gripper_speed(2000)
    arm.set_gripper_position(850,wait=True)
    marker_length = size  # Length of the marker's side in meters
    # print("inloop")
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

            # print(f"Target ArUco tag detected!")
            # print(f"Camera 1 - Rotation Vector:\n{rvec1}\nTranslation Vector:\n{tvec1}")
            # print(f"Camera 2 - Rotation Vector:\n{rvec2}\nTranslation Vector:\n{tvec2}")

            avg_rvec = average_rotation_vectors([rvec1, rvec2])
            avg_tvec = np.mean([tvec1, tvec2], axis=0)

            # print(f"Averaged Rotation Vector:\n{avg_rvec}\nAveraged Translation Vector:\n{avg_tvec}")
            avg_rvec_text = f"Avg Rvec: {avg_rvec[0][0]:.2f}, {avg_rvec[1][0]:.2f}, {avg_rvec[2][0]:.2f}"
            avg_tvec_text = f"Avg Tvec: {avg_tvec[0][0]:.2f}, {avg_tvec[1][0]:.2f}, {avg_tvec[2][0]:.2f}"
            
            mem=avg_tvec.flatten()
            rot=avg_rvec.flatten()
            mem=mem.tolist()
            mem.extend([-180,0,0])
            mem[0]=-mem[0]*1000
            mem[1]=mem[1]*1000
            mem[2]=400
            place,rotation=find_pos(arm,mem,pipeline,tag_id)
            # gohome()
            # print("Place found: ",place)
            return place,rotation


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
def drop_element_at_position(arm,pos):
    drop_claw(arm,pos)
def center_robot_to_laser(cams,laz_pos):
    move_to(arm,laz_pos)
    cap1,cap2,cap3,pipeline=cams
    centered=False
    # save_folder = 'test_v3'
    # if not os.path.exists(save_folder):
    #     os.makedirs(save_folder)

    # Initialize a counter for frame numbering
    # frame_count = 0

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

    leave=False

    while leave is False:
        ret, frame = cap3.read()
        if not ret:
            print("Error: Could not read frame.")
            break
        centered=center_arm(arm,frame)
        if centered==-1:
            break
        elif centered==1:
            # save_frame(frame)
            leave=True
    if leave:
        code,place=arm.get_position_aa(is_radian=False)
        return place
    else:
        return -1
def offset_coor(boxes,coor):
    '''
    offset a coordinate by how many elements between
    '''
    return [coor[0]]+[coor[1]-boxes*105]+coor[2:]
def gohome():
    code,place=arm.get_position_aa(is_radian=False)
    code = arm.set_position_aa(place[:2]+[400]+place[3:], speed=80,mvacc=100, wait=True)
    code=arm.set_servo_angle(angle=[180,75,-180,20,0,90,-60],speed=60,is_radian=False,wait=True)
def readings(cams,step_size,steps):
    cap1,cap2,cap3,pipeline=cams
    centered=False
    centers=[]
    save_folder = 'Full_pipeline'
    if not os.path.exists(save_folder):
        os.makedirs(save_folder)

    # Initialize a counter for frame numbering
    frame_count = 0

    def save_frame(frame,frame_count):

        # Create the filename with frame number
        filename = os.path.join(save_folder, f'frame_{frame_count:04d}.png')  # Saves as frame_0000.png, frame_0001.png, ...

        # Save the current frame to the folder
        cv2.imwrite(filename, frame)
        cv2.imshow('Camera 1',frame)
        cv2.waitKey(1000)
        # Increment the frame counter
        frame_count += 1

    for step in range(steps):
        leave=False

        while leave is False:
            ret, frame = cap3.read()
            if not ret:
                print("Error: Could not read frame.")
                break
            centered=center_arm(arm,frame)
            if centered==-1:
                break
            elif centered==1:
                save_frame(frame,frame_count)
                frame_count+=1
                leave=True
        if leave:
            code,place=arm.get_position_aa(is_radian=False)
            centers.append(place)
        else:
            break
        code,place=arm.get_position_aa(is_radian=False)
        code = arm.set_position_aa([place[0]]+[place[1]-step*step_size]+place[2:], speed=50,mvacc=100, wait=True)
    coordinates_array = np.array(centers)
    return coordinates_array
def full_pipeline(camera_tag,lens_tag):
    start()
    cap1,cap2,cap3,pipeline=initialize_cams()
    cams=(cap1,cap2,cap3,pipeline)

    # pickup camera
    cam_init_pos=pickup_element_with_tag(cams,camera_tag)

    # find laser 
    camera_coor=[343.797485, 386.645844, 264.287628, -129.934337, 124.828596, 0.956668]    
    lens_pos=center_robot_to_laser(cams,camera_coor)
    if lens_pos==-1:
        print("Failed during finding")
        raise Exception
    gohome()
    # drop camera back
    drop_element_at_position(arm,cam_init_pos)
    # pickup lens and put in front of camera
    pickup_element_with_tag(cams,lens_tag)
    drop_element_at_position(arm,lens_pos)

    # move camera to new position and center to beam
    new_cam_pos=offset_coor(1,lens_pos)
    pickup_element_with_tag(cams,camera_tag)
    new_cam_pos=center_robot_to_laser(cams,new_cam_pos)
    
    reading=readings(cams,6,14)
    cv2.destroyAllWindows()

    drop_element_at_position(arm,cam_init_pos)
    gohome()
    pipeline.stop()
    return
def FAT(cams,tag_id,size=0.062):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()
    cap1,cap2,cap3,pipeline=cams
    calibration_data = np.load('stereo_calibration2.npz')
    mtx1 = calibration_data['mtx1']
    dist1 = calibration_data['dist1']
    mtx2 = calibration_data['mtx2']
    dist2 = calibration_data['dist2']
    target_id = tag_id
    arm.set_gripper_enable(True)
    code = arm.set_gripper_speed(2000)
    arm.set_gripper_position(850,wait=True)
    marker_length = size  # Length of the marker's side in meters
    print("inloop")
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
            avg_rvec_text = f"Avg Rvec: {avg_rvec[0][0]:.2f}, {avg_rvec[1][0]:.2f}, {avg_rvec[2][0]:.2f}"
            avg_tvec_text = f"Avg Tvec: {avg_tvec[0][0]:.2f}, {avg_tvec[1][0]:.2f}, {avg_tvec[2][0]:.2f}"
            
            mem=avg_tvec.flatten()
            rot=avg_rvec.flatten()
            mem=mem.tolist()
            mem.extend([-180,0,0])
            mem[0]=-mem[0]*1000
            mem[1]=mem[1]*1000
            mem[2]=400
            pos=None
            move_to(arm,mem)
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
                # print(corners, "corners")

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
            code = arm.set_position_aa([place[0]]+[place[1]]+place[2:], speed=50,mvacc=100, wait=True)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
def rotate_coordinate_plane(angle_degrees):
    """
    Rotate the coordinate plane clockwise by the given angle.
    
    :param angle_degrees: Angle of rotation in degrees (clockwise)
    :return: Rotated unit vectors [x_unit_vector, y_unit_vector]
    """
    # Convert angle to radians
    angle_radians = np.deg2rad(angle_degrees)
    
    # Rotation matrix for clockwise rotation
    rotation_matrix = np.array([
        [np.cos(angle_radians), np.sin(angle_radians)],
        [-np.sin(angle_radians), np.cos(angle_radians)]
    ])
    
    # Original unit vectors
    x_unit = np.array([1, 0])
    y_unit = np.array([0, 1])
    
    # Rotate unit vectors
    rotated_x = rotation_matrix @ x_unit
    rotated_y = rotation_matrix @ y_unit
    
    return [rotated_x, rotated_y]
def calculate_approach_vector(coor,rotation):
    """
    Calculate a unit vector to approach the tag based on its orientation.
    
    Parameters:
    - tag_pose: List of [x, y, z, roll, pitch, yaw].
    - offset_distance: Distance from the tag for starting the approach.

    Returns:
    - start_pose: The starting position before the approach.
    - approach_vector: Unit vector pointing towards the tag.
    """
    add_z=0
    Delx=0
    print(rotation,"rotation in approach")
    # add_z=-0.3+0.02*rotation
    # Delx=1.8+0.01*rotation
    # if rotation>280:
    #     print("hither changing")
    #     add_z=-1.3
    #     Delx=-0.2+0.01*(180-rotation)
    # elif rotation>180:
    #     add_z=1
    #     Delx=-1.2+0.01*(180-rotation)
    # elif rotation>90:
    #     add_z=1.1
    #     Delx=2.4-0.01*(rotation)
    # elif rotation<260:
    #     add_z=3.2
    #     Delx=1
    # elif rotation<=360:
    #     Delx=0.6 
    x_uv,y_uv=rotate_coordinate_plane(rotation)
    # print(x_uv*(Delx)+y_uv*(-113),"additive part")
    x,y=x_uv*(Delx)+y_uv*(-126)
    #-166.4
    #-81.4+16
    #-197
    #-10
    tag_pos_fat=[coor[0]+x]+[coor[1]+y]+[coor[2]-20+add_z]+coor[3:]
    # print(rotation)
    dir_move= np.concatenate([y_uv, np.zeros(4)])
    return tag_pos_fat,dir_move

def move_along_vector(arm, start_pose, direction, distance=3):
    """
    Move the robot arm along a vector in steps.

    Parameters:
    - arm: The robot arm object.
    - start_pose: Starting pose of the robot arm.
    - vector: Unit vector for direction of movement.
    - steps: Number of steps to approach the tag.
    - step_distance: Distance moved in each step (mm).
    """
    # Normalize the vector for stepwise movement
    # print(distance*direction)
    next_pose=start_pose+distance*direction
    arm.set_position_aa(
            next_pose,
            speed=10,
            mvacc=20,
            wait=True
        )
 

def detect_white_squares(frame, min_size=10, max_size=1000, threshold=130):
    """
    Detect largest white square in a frame and return its center point and area.
    
    Args:
        frame: Current frame from video stream
        min_size (int): Minimum side length of squares to detect
        max_size (int): Maximum side length of squares to detect
        threshold (int): Threshold for white color detection (0-255)
    
    Returns:
        tuple: ((center_x, center_y), area) of largest square, or None if no squares found
    """
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Threshold the image to get white regions
    _, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
    
    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    largest_square = None
    largest_area = 0
    
    for contour in contours:
        # Get the bounding rectangle
        x, y, w, h = cv2.boundingRect(contour)
        
        # Check if the contour is approximately square
        aspect_ratio = float(w)/h
        if 0.9 <= aspect_ratio <= 1.1:  # Allow for some deviation from perfect square
            
            # Check if the size is within our target range
            if min_size <= w <= max_size and min_size <= h <= max_size:
                
                # Verify that the region is consistently white
                roi = gray[y:y+h, x:x+w]
                if np.mean(roi) > threshold:
                    area = w * h
                    if area > largest_area:
                        largest_area = area
                        center_x = x + w/2
                        center_y = y + h/2
                        largest_square = ((center_x, center_y), area)
    
    return largest_square
def process_video_stream(cap):
    """
    Process webcam stream and detect white squares in real-time.
    Press 'q' to quit.
    """
    # if not cap.isOpened():
    #     raise ValueError("Could not open video stream")
    
    try:
        while True:
            # Read frame
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            
            # Detect squares
            result = detect_white_squares(frame)
            
            if result is not None:
                (center, area) = result
                center_x, center_y = center
                # Calculate the side length from area
                side_length = int(math.sqrt(area))
                # Calculate top-left corner from center point
                x = int(center_x - side_length/2)
                y = int(center_y - side_length/2)
                
                # Draw rectangle around detected square
                cv2.rectangle(frame, 
                            (x, y), 
                            (x + side_length, y + side_length), 
                            (0, 255, 0), 
                            2)
                
                # Display coordinates (using center coordinates)
                cv2.putText(frame, 
                          f'({int(center_x)},{int(center_y)})', 
                          (x, y-5),
                          cv2.FONT_HERSHEY_SIMPLEX, 
                          0.5, 
                          (0, 255, 0), 
                          1)
                return center_x,center_y
            # Show the frame
            cv2.imshow('White Square Detector', frame)
            
            # Break loop on 'q' press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        # Clean up
        cv2.destroyAllWindows()

def center_fat(frame,rotation):
    

    corners, id1 = detect_aruco(frame,12)
    if corners is not None:
        center_x = np.mean(corners[:, 0])
        center_y = np.mean(corners[:, 1])
        
        # Draw rectangle using the first and third corners
        # Note: This assumes corners are ordered properly
        cv2.rectangle(frame, 
                    (int(corners[0][0]), int(corners[0][1])), 
                    (int(corners[2][0]), int(corners[2][1])), 
                    (0, 255, 0), 
                    2)
        
        # Display coordinates (using center coordinates)
        movex=(1895-center_x)/20 # greater IS TO THE left
        movez=(1235-center_y)/20
        print("center found",center_x,center_y,"and move ",movex,movez)
        if abs(movez)<=0.1 and abs(movex)<=0.1:
            return 1
        print(rotation, "--------------------------------------------------")
        x_basis,y_basis=rotate_coordinate_plane(rotation)
        result=movex * x_basis
        print("basisx ",x_basis)
        print("basisy ",y_basis)
        print("movement ",result)
        code,place=arm.get_position_aa(is_radian=False)
        target_move=[place[0]-result[0]/2]+[place[1]-result[1]/2]+[place[2]+movez/2]+place[3:]
        
        code = arm.set_position_aa(target_move, speed=20,mvacc=100, wait=True)
        return 0
    else:
        return -1  
    
# def center_fat(frame,rotation):
#     # check_saturation(frame)
#     result=detect_white_squares(frame)
#     if result is not None:
#         (center, area) = result
#         center_x, center_y = center
#         print(center)
#         # Calculate the side length from area
#         side_length = int(math.sqrt(area))
#         # Calculate top-left corner from center point
#         x = int(center_x - side_length/2)
#         y = int(center_y - side_length/2)
        
#         # Draw rectangle around detected square
#         cv2.rectangle(frame, 
#                     (x, y), 
#                     (x + side_length, y + side_length), 
#                     (0, 255, 0), 
#                     2)
        
#         # Display coordinates (using center coordinates)
#         cv2.putText(frame,  
#                     f'({int(center_x)},{int(center_y)})', 
#                     (x, y-5),
#                     cv2.FONT_HERSHEY_SIMPLEX, 
#                     0.5, 
#                     (0, 255, 0), 
#                     1)
        
#         movex=(930-center_x)/20 # 1020 IS TO THE left
#         movez=(290-center_y)/20
#         print("center found",center_x,center_y,"and move ",movex,movez)
#         if abs(movez)<=0.1 and abs(movex)<=0.1:
#             return 1
#         print(rotation, "--------------------------------------------------")
#         x_basis,y_basis=rotate_coordinate_plane(rotation)
#         result=movex * x_basis
#         print("basisx ",x_basis)
#         print("basisy ",y_basis)
#         print("movement ",result)
#         code,place=arm.get_position_aa(is_radian=False)
#         target_move=[place[0]-result[0]/2]+[place[1]-result[1]/2]+[place[2]+movez/2]+place[3:]
        
#         code = arm.set_position_aa(target_move, speed=20,mvacc=100, wait=True)
#         return 0
#     else:
#         return -1   
#     return None
#     # Show the frame
            
def move_along_vector_feedback(fatcam,arm,place,dir_move1,rotation,distance=18):


    """
    Move the robot arm along a vector in steps.

    Parameters:
    - arm: The robot arm object.
    - start_pose: Starting pose of the robot arm.
    - vector: Unit vector for direction of movement.
    - steps: Number of steps to approach the tag.
    - step_distance: Distance moved in each step (mm).
    """
    current_steps=distance-1
    center=None
    while True:
        ret1, frame1 = fatcam.read()
        if not ret1:
            print("Failed to grab frame")
            break
        
        # Make a copy of the frame for display
        display_frame = frame1.copy()
        
        # Process frame with center_fat
        centered = center_fat(display_frame,rotation)
        
        # Show the processed frame
        cv2.imshow('Camera Feed', display_frame)
        
        # Add wait key for proper display and exit condition
        key = cv2.waitKey(1)
        if key == ord('q'):  # Press 'q' to quit
            break
            
        if centered == 1:
            print("Target centered")
            code,place=arm.get_position_aa(is_radian=False)
            move_along_vector(arm,place,dir_move1,distance)
            return
        elif centered == -1:
            print("No target detected")
        
        # Optional delay to make the visualization more visible
        time.sleep(0.1)

    def __init__(self, port='COM10', baud_rate=115200):

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
    
# if __name__ == '__main__':
#     # controller = StepperController()

#     camera_tag=2
#     lens_tag=2
#     start()
#     cap1,cap2,cap3,pipeline=initialize_cams()
#     cams=(cap1,cap2,cap3,pipeline)
#     fatcam = cv2.VideoCapture(3, cv2.CAP_MSMF)
#     fatcam.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
#     fatcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
#     ######################### Finding position ########################
#     # def save_calibration(filename, tagpos1, tagpos2, tagpos3, tagpos4, 
#     #                  rotation1, rotation2, rotation3, rotation4):
#     #     np.savez(filename,
#     #             tagpos1=tagpos1,
#     #             tagpos2=tagpos2,
#     #             tagpos3=tagpos3,
#     #             tagpos4=tagpos4,
#     #             rotation1=rotation1,
#     #             rotation2=rotation2,
#     #             rotation3=rotation3,
#     #             rotation4=rotation4)
#     #     print(f"Saved calibration data to {filename}")

#     # tag_pos1,rotation1=find_position_of_tag(cams,8)
#     # gohome()
#     # tag_pos2,rotation2=find_position_of_tag(cams,9)
#     # gohome()
#     # tag_pos3,rotation3=find_position_of_tag(cams,10)
#     # gohome()
#     # tag_pos4,rotation4=find_position_of_tag(cams,11)
#     # gohome()
#     # print([tag_pos1,tag_pos2,tag_pos3,tag_pos4])
#     # print([rotation1,rotation2,rotation3,rotation4])
#     # save_calibration("interferometer.npz", 
#     #              tag_pos1, tag_pos2, tag_pos3, tag_pos4,
#     #              rotation1, rotation2, rotation3, rotation4)

#     ###################### Placing Elements #########################

#     def load_calibration(filename):
#         data = np.load(filename)
        
#         # Convert NumPy arrays to Python lists
#         tagpos1 = data['tagpos1'].tolist()
#         tagpos2 = data['tagpos2'].tolist()
#         tagpos3 = data['tagpos3'].tolist()
#         tagpos4 = data['tagpos4'].tolist()
        
#         # For scalar values, use item() to convert to a Python scalar
#         rotation1 = data['rotation1'].item() if isinstance(data['rotation1'], np.ndarray) else data['rotation1']
#         rotation2 = data['rotation2'].item() if isinstance(data['rotation2'], np.ndarray) else data['rotation2']
#         rotation3 = data['rotation3'].item() if isinstance(data['rotation3'], np.ndarray) else data['rotation3']
#         rotation4 = data['rotation4'].item() if isinstance(data['rotation4'], np.ndarray) else data['rotation4']
        
#         return (tagpos1, tagpos2, tagpos3, tagpos4,
#                 rotation1, rotation2, rotation3, rotation4)
#     tag_pos1, tag_pos2, tag_pos3, tag_pos4, rotation1, rotation2, rotation3, rotation4 = load_calibration("interferometer.npz")

#     temp_pos,temp_rotation=pickup_element_with_tag(cams,8)
#     drop_element_at_position(arm,tag_pos1[:2]+[temp_pos[2]]+tag_pos1[3:])

#     temp_pos,temp_rotation=pickup_element_with_tag(cams,10)
#     drop_element_at_position(arm,tag_pos3[:2]+[temp_pos[2]]+tag_pos3[3:])

#     temp_pos,temp_rotation=pickup_element_with_tag(cams,9)
#     drop_element_at_position(arm,tag_pos2[:2]+[temp_pos[2]]+tag_pos2[3:])

#     temp_pos,temp_rotation=pickup_element_with_tag(cams,11)
#     drop_element_at_position(arm,tag_pos4[:2]+[temp_pos[2]]+tag_pos4[3:])





#     tag_pos4,rotation4=find_position_of_tag(cams,11) 
#     tag_pos_fat1,dir_move1=calculate_approach_vector(tag_pos4,(rotation4+90)%360)
#     fat_position,rotation=pickup_element_with_tag(cams,1,0.038,True)
#     move_to(arm,tag_pos_fat1)
#     code,place=arm.get_position_aa(is_radian=False)
    
#     # # # print("Enabling motors...")
#     # controller.move_all_motors(2*4096, -2*4096, 2*4096)
#     move_along_vector(arm,place,dir_move1)
#     move_along_vector_feedback(fatcam,arm,place,dir_move1,(rotation1+90)%360)
#     move_along_vector(arm,place,-dir_move1,25)
#     drop_element_at_position(arm,fat_position)
#     gohome()
#     # controller.power_motor(0, False)

#     ########################### DEBUG ############################################

#     # while True:
#     #     ret1, frame1 = fatcam.read()
#     #     center_fat(frame1,rotation1)
#     #     cv2.imshow('Camera Feed', frame1)
#     #     key = cv2.waitKey(1)
#     #     if key == ord('q'):  # Press 'q' to quit
#     #         break



#     # # move_along_vector(arm,place,dir_move4)
#     # # # # time.sleep(3)
#     # # # # # rotate_motor_async(board)
#     # # code,place=arm.get_position_aa(is_radian=False)
#     # # move_along_vector(arm,place,-dir_move4)
#     # # code,pos=arm.get_position_aa(is_radian=False)
#     # # code = arm.set_position_aa(pos[:2]+[450]+pos[3:], speed=40,mvacc=40, wait=True)
#     # # gohome()
#     # # drop_element_at_position(arm,fat_position)
#     # ############################################################################
#     # # board.exit()
#     # controller.close()



if __name__ == '__main__':
    # controller = StepperController()

    camera_tag=2
    lens_tag=2
    start()
    cap1,cap2,cap3,pipeline=initialize_cams()
    cams=(cap1,cap2,cap3,pipeline)
    fatcam = cv2.VideoCapture(3, cv2.CAP_MSMF)
    fatcam.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    fatcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
    ######################### Finding position ########################
    # def save_calibration(filename, tagpos1, tagpos2, tagpos3, tagpos4,
    #                  rotation1, rotation2, rotation3, rotation4):
    #     np.savez(filename,
    #             tagpos1=tagpos1,
    #             tagpos2=tagpos2,
    #             tagpos3=tagpos3,
    #             tagpos4=tagpos4,
    #             rotation1=rotation1,
    #             rotation2=rotation2,
    #             rotation3=rotation3,
    #             rotation4=rotation4)
    #     print(f"Saved calibration data to {filename}")

    # tag_pos1,rotation1=find_position_of_tag(cams,8)
    # gohome()
    # tag_pos2,rotation2=find_position_of_tag(cams,9)
    # gohome()
    # tag_pos3,rotation3=find_position_of_tag(cams,10)
    # gohome()
    # tag_pos4,rotation4=find_position_of_tag(cams,11)
    # gohome()
    # print([tag_pos1,tag_pos2,tag_pos3,tag_pos4])
    # print([rotation1,rotation2,rotation3,rotation4])
    # save_calibration("interferometer.npz",
    #              tag_pos1, tag_pos2, tag_pos3, tag_pos4,
    #              rotation1, rotation2, rotation3, rotation4)

    ###################### Placing Elements #########################

    epsilon=5
   
    def load_calibration(filename):
        data = np.load(filename)
       
        # Convert NumPy arrays to Python lists
        tagpos1 = data['tagpos1'].tolist()
        tagpos2 = data['tagpos2'].tolist()
        tagpos3 = data['tagpos3'].tolist()
        tagpos4 = data['tagpos4'].tolist()
       
        # For scalar values, use item() to convert to a Python scalar
        rotation1 = data['rotation1'].item() if isinstance(data['rotation1'], np.ndarray) else data['rotation1']
        rotation2 = data['rotation2'].item() if isinstance(data['rotation2'], np.ndarray) else data['rotation2']
        rotation3 = data['rotation3'].item() if isinstance(data['rotation3'], np.ndarray) else data['rotation3']
        rotation4 = data['rotation4'].item() if isinstance(data['rotation4'], np.ndarray) else data['rotation4']
       
        return (tagpos1, tagpos2, tagpos3, tagpos4,
                rotation1, rotation2, rotation3, rotation4)
    tag_pos1, tag_pos2, tag_pos3, tag_pos4, rotation1, rotation2, rotation3, rotation4 = load_calibration("interferometer.npz")

    temp_pos,temp_rotation=pickup_element_with_tag(cams,8)
    drop_element_at_position(arm,tag_pos1[:2]+[temp_pos[2]]+tag_pos1[3:])
    while(True):
        tag_pos1_found,rotation1_found=find_position_of_tag(cams,8)
        gohome()
        print(np.abs(tag_pos1_found[0]-tag_pos1[0]),np.abs(tag_pos1_found[1]-tag_pos1[1]),np.abs(rotation1-rotation1_found))
        if (np.abs(tag_pos1_found[0]-tag_pos1[0])<epsilon and np.abs(tag_pos1_found[1]-tag_pos1[1])<epsilon):
            if (np.abs(rotation1-rotation1_found)<epsilon):
                break;
        print("Readjusting due to x error: ",tag_pos1_found[0]-tag_pos1[0]," y error: ",tag_pos1_found[1]-tag_pos1[1])
        temp_pos,temp_rotation=pickup_element_with_tag(cams,8)
        drop_element_at_position(arm,tag_pos1[:2]+[temp_pos[2]]+tag_pos1[3:])

    temp_pos,temp_rotation=pickup_element_with_tag(cams,10)
    drop_element_at_position(arm,tag_pos3[:2]+[temp_pos[2]]+tag_pos3[3:])
    while(True):
        tag_pos3_found,rotation3_found=find_position_of_tag(cams,10)
        gohome()
        print(np.abs(tag_pos3_found[0]-tag_pos3[0]),np.abs(tag_pos3_found[1]-tag_pos3[1]),np.abs(rotation3-rotation3_found))

        if (np.abs(tag_pos3_found[0]-tag_pos3[0])<epsilon and np.abs(tag_pos3_found[1]-tag_pos3[1])<epsilon):
            if (np.abs(rotation3-rotation3_found)<epsilon):
                break;
        print("Readjusting due to x error: ",tag_pos3_found[0]-tag_pos3[0]," y error: ",tag_pos3_found[1]-tag_pos3[1])
        temp_pos,temp_rotation=pickup_element_with_tag(cams,10)
        drop_element_at_position(arm,tag_pos3[:2]+[temp_pos[2]]+tag_pos3[3:])

    temp_pos,temp_rotation=pickup_element_with_tag(cams,9)
    drop_element_at_position(arm,tag_pos2[:2]+[temp_pos[2]]+tag_pos2[3:])
    while(True):
        tag_pos2_found,rotation2_found=find_position_of_tag(cams,9)
        gohome()
        print(np.abs(tag_pos2_found[0]-tag_pos2[0]),np.abs(tag_pos2_found[1]-tag_pos2[1]),np.abs(rotation2-rotation2_found))

        if (np.abs(tag_pos2_found[0]-tag_pos2[0])<epsilon and np.abs(tag_pos2_found[1]-tag_pos2[1])<epsilon):
            if (np.abs(rotation2-rotation2_found)<10):
                break;
        print("Readjusting due to x error: ",tag_pos2_found[0]-tag_pos2[0]," y error: ",tag_pos2_found[1]-tag_pos2[1])
        temp_pos,temp_rotation=pickup_element_with_tag(cams,9)
        drop_element_at_position(arm,tag_pos2[:2]+[temp_pos[2]]+tag_pos2[3:])
    # print("HEREEEEEEEEEEEEEEEEEEEEEEE")
    temp_pos,temp_rotation=pickup_element_with_tag(cams,11)
    drop_element_at_position(arm,tag_pos4[:2]+[temp_pos[2]]+tag_pos4[3:])

    while(True):
        tag_pos4_found,rotation4_found=find_position_of_tag(cams,11)
        gohome()
        print(np.abs(tag_pos4_found[0]-tag_pos4[0]),np.abs(tag_pos4_found[1]-tag_pos4[1]),np.abs(rotation4-rotation4_found))

        if (np.abs(tag_pos4_found[0]-tag_pos4[0])<epsilon and np.abs(tag_pos4_found[1]-tag_pos4[1])<epsilon):
            if (np.abs(rotation4-rotation4_found)<epsilon):
                break;
        print("Readjusting due to x error: ",tag_pos4_found[0]-tag_pos4[0]," y error: ",tag_pos4_found[1]-tag_pos4[1])
        temp_pos,temp_rotation=pickup_element_with_tag(cams,11)
        drop_element_at_position(arm,tag_pos4[:2]+[temp_pos[2]]+tag_pos4[3:])


    # tag_pos4,rotation4=find_position_of_tag(cams,11)
    # tag_pos_fat1,dir_move1=calculate_approach_vector(tag_pos4,(rotation4+90)%360)
    # fat_position,rotation=pickup_element_with_tag(cams,1,0.038,True)
    # move_to(arm,tag_pos_fat1)
    # code,place=arm.get_position_aa(is_radian=False)
   
    # # # # print("Enabling motors...")
    # # controller.move_all_motors(2*4096, -2*4096, 2*4096)
    # move_along_vector(arm,place,dir_move1)
    # move_along_vector_feedback(fatcam,arm,place,dir_move1,(rotation1+90)%360)
    # move_along_vector(arm,place,-dir_move1,25)
    # drop_element_at_position(arm,fat_position)
    # gohome()






    # controller.power_motor(0, False)

    ########################### DEBUG ############################################

    # while True:
    #     ret1, frame1 = fatcam.read()
    #     center_fat(frame1,rotation1)
    #     cv2.imshow('Camera Feed', frame1)
    #     key = cv2.waitKey(1)
    #     if key == ord('q'):  # Press 'q' to quit
    #         break



    # # move_along_vector(arm,place,dir_move4)
    # # # # time.sleep(3)
    # # # # # rotate_motor_async(board)
    # # code,place=arm.get_position_aa(is_radian=False)
    # # move_along_vector(arm,place,-dir_move4)
    # # code,pos=arm.get_position_aa(is_radian=False)
    # # code = arm.set_position_aa(pos[:2]+[450]+pos[3:], speed=40,mvacc=40, wait=True)
    # # gohome()
    # # drop_element_at_position(arm,fat_position)
    # ############################################################################
    # # board.exit()
    # controller.close()