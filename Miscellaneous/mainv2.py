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
    # RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    print(cv2.__version__)
    # Load calibration data
    calibration_data = np.load('stereo_calibration2.npz')
    mtx1 = calibration_data['mtx1']
    dist1 = calibration_data['dist1']
    mtx2 = calibration_data['mtx2']
    dist2 = calibration_data['dist2']
    R = calibration_data['R']
    T = calibration_data['T']
    actual_distance = 0.23  # 23 cm
    calibrated_distance = 0.22  # 22 cm

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
    # Main loop for real-time detection
    cap1 = cv2.VideoCapture(1, cv2.CAP_MSMF)
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
    cap2 = cv2.VideoCapture(2, cv2.CAP_MSMF)
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
    target_id = 4
    marker_length = 0.062  # Length of the marker's side in meters
    start()

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

            # draw_axis(frame1, rvec1, tvec1, mtx1, dist1)
            # draw_axis(frame2, rvec2, tvec2, mtx2, dist2)

            avg_rvec = average_rotation_vectors([rvec1, rvec2])
            avg_tvec = np.mean([tvec1, tvec2], axis=0)

            print(f"Averaged Rotation Vector:\n{avg_rvec}\nAveraged Translation Vector:\n{avg_tvec}")

            draw_axis(frame1, avg_rvec, avg_tvec, mtx1, dist1)

            avg_rvec_text = f"Avg Rvec: {avg_rvec[0][0]:.2f}, {avg_rvec[1][0]:.2f}, {avg_rvec[2][0]:.2f}"
            avg_tvec_text = f"Avg Tvec: {avg_tvec[0][0]:.2f}, {avg_tvec[1][0]:.2f}, {avg_tvec[2][0]:.2f}"
            
            # Draw the text on the frame
            # cv2.putText(frame1, avg_rvec_text, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            # cv2.putText(frame1, avg_tvec_text, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            mem=avg_tvec.flatten()
            rot=avg_rvec.flatten()
            mem=mem.tolist()
            pitch=extract_pitch_from_rotation(avg_rvec)
            addend = extract_euler_angles(avg_rvec)
            mem.extend([-180,0,0])
            mem[0]=-mem[0]*1000
            mem[1]=mem[1]*1000
            mem[2]=(1.2-mem[2])*1000
            pickup(arm,mem,pitch)
        # cv2.namedWindow('Camera 1', cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('Camera 1', 3840, 2160)
        # cv2.imshow('Camera 1', frame1)
        # cv2.namedWindow('Camera 2', cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('Camera 2', 3840, 2160)
        # cv2.imshow('Camera 2', frame2)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()
