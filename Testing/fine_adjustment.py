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
# arm.reset(wait=True)
print("init done")
# print("1")
# arm = XArmAPI(ip,is_radian=False)
# arm.motion_enable(enable=True)
# arm.set_mode(1)
# print("passed")
# arm.set_state(state=0)
# arm.clean_error()
# arm = XArmAPI(ip, is_radian=False)
# arm.motion_enable(enable=True)
# arm.set_mode(1)
# arm.set_state(state=0)
# arm.reset(wait=True)
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

def pickup(arm,coor,pitch):
    print(coor,"pickup")
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
        
        
    # coor[1]=coor[1]-20
    # elif coor[0]>70 and coor[0]<170:
    #     coor[0]=coor[0]*0.02*(coor[0]-70)
    # elif coor[0]>20:
    #     coor[0]=coor[0]*0.90
    print(coor[1])

    highcoor=coor[:2]+[330]+coor[3:]
    code = arm.set_servo_angle(servo_id=1,angle=new_angle,wait=True,is_radian=False,speed=speeds)
    code = arm.set_position_aa(highcoor, speed=speeds,mvacc=100, wait=True)
    code = arm.set_servo_angle(servo_id=7,angle=pitch,wait=True,is_radian=False,speed=speeds)
    code,place=arm.get_position_aa(is_radian=False)
    arm.set_vacuum_gripper(on=True)
    arm.set_tcp_load(weight=0.8, center_of_gravity=(0.06125, 0.0458, 0.0375))
    print(pitch,"pitch")
    val=0.8
    i=1
    while True:
        code = arm.set_position_aa(place[:2]+[330-(2*i)]+place[3:], speed=speeds,mvacc=100, wait=True)
        if i==1:
            if sum(arm.currents)<1.5:
                val=-1
            elif sum(arm.currents)>4:
                val=1.5
        ans=sum(arm.currents)
        print(ans,"*****",val)
        if ans<val:
            print("here")
            break
        i+=1
        
    code = arm.set_position_aa(place[:2]+[400]+place[3:], speed=speeds,mvacc=100, wait=True)
    code = arm.set_servo_angle(angle=[180,75,-180,20,0,90,-60],is_radian=False,speed=speeds)

    print(quad)
    return 330-2*i

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
    # if not self._check_code(code, 'set_servo_angle'):
    #     return
    code = arm.set_position_aa(highcoor,is_radian=False, speed=100,  mvacc=100, wait=True)
    # if not self._check_code(code, 'set_position'):
    #     return

    code = arm.set_position_aa(coor,is_radian=False, speed=80,  mvacc=100, wait=True)
    arm.set_vacuum_gripper(on=False)
    arm.set_tcp_load(weight=0.61, center_of_gravity=(0.06125, 0.0458, 0.0375))

    # if not self._check_code(code, 'set_position'):
    #     return
    # code = arm.set_gripper_position(800, wait=True, speed=5000, auto_enable=True)
    # if not self._check_code(code, 'set_gripper_position'):
    #     return
    code = arm.set_position_aa(highcoor,is_radian=False, speed=100,  mvacc=100, wait=True)
    # if not self._check_code(code, 'set_position'):
    #     return
    code=arm.set_servo_angle(angle=[180,75,-180,20,0,90,-60],speed=100,is_radian=False,wait=True)
    # if not self._check_code(code, 'set_servo_angle'):
    #     return

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


if __name__ == '__main__':
    # Initialize RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth,640,480,rs.format.z16,15)

    pipeline.start(config)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()
    target_id=4
    leave=False
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
            cv2.imshow("image",depth_image)
            cv2.waitKey(1000)
    code = arm.set_position_aa([place[0]+75]+[place[1]+35]+place[2:], speed=50,mvacc=100, wait=True)
    depth_value = depth_image[center_y, center_x]
    print(depth_value)
    pipeline.stop()
    cv2.destroyAllWindows()
