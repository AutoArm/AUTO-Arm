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
    code = arm.set_position_aa([place[0]+72]+[place[1]+37]+place[2:], speed=50,mvacc=100, wait=True)
    code,pos = arm.get_servo_angle(servo_id=7,is_radian=False)
    code = arm.set_servo_angle(servo_id=7,wait=True,angle=pos+rotation_angle,is_radian=False)


    print("depth",depth_value)
    return depth_value

if __name__ == '__main__':
    print(cv2.__version__)
    # Load calibration data
    # temp=arm.get_position(is_radian=False)
    # print(temp, " is base")

    highdrop=[6.759449, 324.608215, 400.279938, 179.688159, 1.214785, -0.9687]
    lowdrop=[6.759449, 324.608215, 259.6, 179.688159, 1.214785, -0.9687]
    # start()
    cap2 = cv2.VideoCapture(1)

    arm.set_gripper_enable(True)
    # code = arm.set_gripper_speed(2000)
    # code = arm.set_position_aa(highdrop,is_radian=False, speed=50,  mvacc=100, wait=True)
    code = arm.set_position_aa(lowdrop,is_radian=False, speed=10,  mvacc=100, wait=True)
    # code = arm.set_position_aa(highdrop,is_radian=False, speed=50,  mvacc=100, wait=True)
    rightcount=0
    print("calib image take in 4 seconds")
    time.sleep(4)
    print("Now")
    for i in range(20):
        ret2, frame2 = cap2.read()
        filename1 = 'images/base3/second'+str(rightcount)+'.jpg'
        cv2.imwrite(filename1, frame2) 
        rightcount+=1
        
        cv2.waitKey(1000)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    rightcount=0
    print("ready for pictures")
    time.sleep(5)
    print("2 seconds")
    time.sleep(2)
    ret2, frame2 = cap2.read()
    filename1 = 'images/try3/'+str(rightcount)+'.jpg'
    cv2.imwrite(filename1, frame2) 
    rightcount+=1
    for i in range(1000):
        code = arm.set_position_aa(highdrop,is_radian=False, speed=10,  mvacc=100, wait=True)
        code = arm.set_position_aa(lowdrop,is_radian=False, speed=10,  mvacc=100, wait=True)
        cv2.waitKey(4000)
        ret2, frame2 = cap2.read()
        filename1 = 'images/try3/'+str(rightcount)+'.jpg'
        cv2.imwrite(filename1, frame2) 
        rightcount+=1
        cv2.imshow('Camera', frame2)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    cap2.release()
    cv2.destroyAllWindows()
