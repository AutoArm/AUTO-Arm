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


    # Robot Main Run
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

def pickup(arm,coor):
    speeds=80
    x=coor[0]
    y=coor[1]
    coor[3]=-180

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
    if quad==4:
        
        if coor[0]>270:
            coor[0]=coor[0]+9.5
        if coor[0]<90:
            coor[0]=coor[0]+(90-coor[0])/50*15
        if coor[0]<110:
            coor[0]=coor[0]+7.5
        print(coor[1])
        coor[1]=coor[1]+coor[1]/110*15
        # coor[1]=coor[1]-20
        # elif coor[0]>70 and coor[0]<170:
        #     coor[0]=coor[0]*0.02*(coor[0]-70)
        # elif coor[0]>20:
        #     coor[0]=coor[0]*0.90
        print(coor[1])

        coor[0]=coor[0]+20
        highcoor=coor[:2]+[330]+coor[3:]
        code = arm.set_servo_angle(servo_id=1,angle=new_angle,wait=True,is_radian=False,speed=speeds)
        code = arm.set_position_aa(highcoor, speed=speeds, mvacc=100, wait=True)
        arm.set_vacuum_gripper(on=True)
        arm.set_tcp_load(weight=0.8, center_of_gravity=(0.06125, 0.0458, 0.0375))

        val=0.8
        i=1
        while True:
            
            code = arm.set_position_aa(highcoor[:2]+[330-(2*i)]+highcoor[3:],is_radian=False,speed=100,wait=True)
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
        code = arm.set_position_aa(highcoor, speed=speeds,mvacc=100, wait=True)
        code = arm.set_servo_angle(angle=[180,75,-180,20,0,90,-60],is_radian=False,speed=speeds)
    elif quad==1:
        highcoor=coor[:2]+[330]+coor[3:]
        code = arm.set_servo_angle(servo_id=1,angle=new_angle,wait=True,is_radian=False,speed=speeds)
        code = arm.set_position_aa(highcoor, speed=speeds, mvacc=100, wait=True)
        arm.set_vacuum_gripper(on=True)
        arm.set_tcp_load(weight=0.8, center_of_gravity=(0.06125, 0.0458, 0.0375))

        i=1
        while True:
            code = arm.set_position_aa(highcoor[:2]+[330-2*i]+highcoor[3:],is_radian=False,speed=70,wait=True)
            ans=sum(arm.currents)
            print(ans)
            if ans<0:
                print("here")
                break
            i+=1

        code = arm.set_position_aa(highcoor, speed=speeds,mvacc=100, wait=True)
        code = arm.set_servo_angle(angle=[180,75,-180,20,0,90,-60],is_radian=False,speed=speeds)
    elif quad==3:
        # if coor[0]>-90:
        #     coor[0]=coor[0]+coor[0]/90*15
        # elif coor[0]>-160:
        #     coor[0]=coor[0]+coor[0]/100*15
        # elif coor[0]>-270:
        #     coor[0]=coor[0]+(coor[0])/130*15
        # elif coor[0]<-270:
        #     coor[0]=coor[0]+(coor[0])/170*15

        
        
        print(coor[1])
        coor[1]=coor[1]+coor[1]/130*15
        # coor[1]=coor[1]-20
        # elif coor[0]>70 and coor[0]<170:
        #     coor[0]=coor[0]*0.02*(coor[0]-70)
        # elif coor[0]>20:
        #     coor[0]=coor[0]*0.90
        print(coor[1])

        coor[0]=coor[0]-42
        highcoor=coor[:2]+[330]+coor[3:]
        code = arm.set_servo_angle(servo_id=1,angle=new_angle,wait=True,is_radian=False,speed=speeds)
        code = arm.set_position_aa(highcoor, speed=speeds, mvacc=100, wait=True)
        arm.set_vacuum_gripper(on=True)
        arm.set_tcp_load(weight=0.8, center_of_gravity=(0.06125, 0.0458, 0.0375))

        val=0
        i=1
        while True:
            
            code = arm.set_position_aa(highcoor[:2]+[330-(2*i)]+highcoor[3:],is_radian=False,speed=100,wait=True)
            if i==1:
                if sum(arm.currents)<1.5:
                    val=-0.5
                elif sum(arm.currents)>3:
                    val=1
                else:
                    val=0.5
            ans=sum(arm.currents)
            print(ans,"*****",val)
            if ans<val:
                print("here")
                break
            i+=1
        code = arm.set_position_aa(highcoor, speed=speeds,mvacc=100, wait=True)
        code = arm.set_servo_angle(angle=[180,75,-180,20,0,90,-60],is_radian=False,speed=speeds)
    else:
        coor[3]=180
        coor[0]=coor[0]-75
        highcoor=coor[:2]+[330]+coor[3:]
        code=arm.set_servo_angle(servo_id=1,angle=new_angle,wait=True,is_radian=False,speed=speeds)
        code = arm.set_position_aa(highcoor, speed=speeds, mvacc=100, wait=True)
        arm.set_vacuum_gripper(on=True)
        arm.set_tcp_load(weight=0.8, center_of_gravity=(0.06125, 0.0458, 0.0375))

        print(sum(arm.currents),"init")
        begin=sum(arm.currents)
        i=1
        while True:
            code = arm.set_position_aa(highcoor[:2]+[330-2*i]+highcoor[3:],is_radian=False,speed=70,wait=True)
            ans=sum(arm.currents)
            print((ans))
            if ans>-1.0:
                print("here")
                break
            i+=1
        code = arm.set_position_aa(highcoor, speed=speeds,mvacc=100, wait=True)
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
    calibration_data = np.load('stereo_calibration.npz')
    mtx1 = calibration_data['mtx1']
    dist1 = calibration_data['dist1']
    mtx2 = calibration_data['mtx2']
    dist2 = calibration_data['dist2']
    R = calibration_data['R']
    T = calibration_data['T']

    # Compute projection matrices
    proj1 = mtx1 @ np.hstack((np.eye(3), np.zeros((3, 1))))
    proj2 = mtx2 @ np.hstack((R, T))

    # Define the ArUco dictionary and parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()

    # Define the ArUco marker size (in meters)
    aruco_size = 0.065

    # Define the 3D points of the ArUco marker corners (assuming square markers)
    aruco_corners_3d = np.array([
        [-aruco_size / 2, -aruco_size / 2, 0],
        [aruco_size / 2, -aruco_size / 2, 0],
        [aruco_size / 2, aruco_size / 2, 0],
        [-aruco_size / 2, aruco_size / 2, 0]
    ], dtype=np.float32)

    # Scaling factor
    scale_factor = 1.22 / 0.74

    # Main loop for real-time detection
    cap1 = cv2.VideoCapture(1, cv2.CAP_MSMF)
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
    cap2 = cv2.VideoCapture(2, cv2.CAP_MSMF)
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
    
    print("staretpre")
    start()
    print("start")
    targets=[4,2]
    target_id = targets[0]
    # target_id=4
    counter=0
    places=[[300,400,0,-180,0,0],[-300,400,0,-180,0,0]]
    while True:
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()

        if not ret1 or not ret2:
            print("Error: Could not read frame from one or both cameras.")
            break

        gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners1, ids1, _ = detector.detectMarkers(gray1)
        corners2, ids2, _ = detector.detectMarkers(gray2)
        pose1=None
        pose2=None
        mult=510
        if ids1 is not None and target_id in ids1.flatten():
            idx1 = np.where(ids1.flatten() == target_id)[0][0]
            corners1_target = corners1[idx1].reshape(-1, 2)

            # Estimate pose using solvePnP
            success1, rvec1, tvec1 = cv2.solvePnP(aruco_corners_3d, corners1_target, mtx1, dist1)

            if success1:
                # Scale and adjust the position
                tvec1 = tvec1 * scale_factor
                

                # Draw the detected marker and axes
                cv2.aruco.drawDetectedMarkers(frame1, corners1)
                cv2.drawFrameAxes(frame1, mtx1, dist1, rvec1, tvec1, 0.1)
                tvec1[2] = 1.52-tvec1[2]
                tvec1[0]=-tvec1[0]
                tvec1[0]=tvec1[0]*mult
                tvec1[1]=tvec1[1]*mult
                # tvec1[1]=tvec1[1]+(tvec1[1]/45)

                trans_x=0
                # if tvec1[0]>=0:
                #     trans_x=math.exp((440-tvec1[0])/80)
                # else:
                #     trans_x=-math.exp((440+tvec1[0])/90)
                # tvec1[0]=tvec1[0]-trans_x

                # trans_y=0
                # if tvec1[1]>=0:
                #     trans_y=math.exp(tvec1[1]/95)
                # else:
                #     trans_y=-math.exp(-tvec1[1]/125)
                # tvec1[1]=tvec1[1]+trans_y

                # tvec1[2]=tvec1[2]*mult
                # tvec1[2]+=90

                # Convert rotation vector to a rotation matrix
                R1, _ = cv2.Rodrigues(rvec1)
                pose_matrix1 = np.hstack((R1, tvec1))

                # Print position and orientation
                print(f"Position (Camera 1): {tvec1.flatten()}")
                pose1=tvec1.flatten()
                print(f"Rotation Matrix (Camera 1):\n{R1}")

        if ids2 is not None and target_id in ids2.flatten():
            idx2 = np.where(ids2.flatten() == target_id)[0][0]
            corners2_target = corners2[idx2].reshape(-1, 2)

            # Estimate pose using solvePnP
            success2, rvec2, tvec2 = cv2.solvePnP(aruco_corners_3d, corners2_target, mtx2, dist2)

            if success2:
                # Scale and adjust the position
                tvec2 = tvec2 * scale_factor

                # Draw the detected marker and axes
                cv2.aruco.drawDetectedMarkers(frame2, corners2)
                cv2.drawFrameAxes(frame2, mtx2, dist2, rvec2, tvec2, 0.1)
                tvec2[2] = 1.3-tvec2[2]
                tvec2[0]=-tvec2[0]
                tvec2[0]=tvec2[0]*mult

                tvec2[1]=tvec2[1]*mult
                trans_x=0
                if tvec2[0]>=0:
                    trans_x=math.exp((440-tvec2[0])/80)
                else:
                    trans_x=-math.exp((440+tvec2[0])/90)
                tvec2[0]=tvec2[0]-trans_x
                
                trans_y=0
                if tvec2[1]>=0:
                    trans_y=math.exp(tvec2[1]/95)
                else:
                    trans_y=-math.exp(-tvec2[1]/125)
                tvec2[1]=tvec2[1]+trans_y

                tvec2[2]=tvec2[2]*mult
                tvec2[2]+=90

                # Convert rotation vector to a rotation matrix
                R2, _ = cv2.Rodrigues(rvec2)
                pose_matrix2 = np.hstack((R2, tvec2))

                # Print position and orientation
                print(f"Position (Camera 2): {tvec2.flatten()}")
                pose2=tvec2.flatten()
                print(f"Rotation Matrix (Camera 2):\n{R2}")
        if pose1 is not None and pose2 is not None:

            rvec_avg, tvec_avg = average_pose(rvec1, tvec1, rvec2, tvec2)
            print(f"Averaged Position:{tvec_avg.flatten()}")
            print(f"Averaged Rotation Vector: {rvec_avg.flatten()}")
            mem=tvec_avg.flatten()
            mem=mem.tolist()
            mem.extend([-180,0,0])
            print("*********************",mem)
            height=pickup(arm,mem)+3
            place=places[counter]
            place[2]=height
            drop(arm,place)
            counter+=1
            if counter==len(targets):
                counter=0
            target_id=targets[counter]
        # Display frames

        # cv2.namedWindow('Camera 1', cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('Camera 1', 3840, 2160)
        # cv2.imshow('Camera 1', frame1)
        # cv2.namedWindow('Camera 2', cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('Camera 2', 3840, 2160)
        # cv2.imshow('Camera 2', frame2)

        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()
