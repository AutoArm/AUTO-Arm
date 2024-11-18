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
from matplotlib import pyplot as plt
import ctypes
from scipy.io import savemat



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
        # hard coded ip; might change
        ip = "192.168.1.241"
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

def y_rot(vec, angle):
    """
    given a vector and an angle (deg), rotates
    the vector about the y-axis 
    """
    angle = np.radians(angle)

    v1 = vec[0]*np.cos(angle) + vec[2]*np.sin(angle)
    v2 = vec[1]
    v3 = vec[2]*np.cos(angle) - vec[0]*np.sin(angle)

    return [v1, v2, v3]

def sub(v1, v2):
    """
    given two vectors, returns v1 - v2 
    """
    new_vec = []

    for i in range(len(v1)):
        new_vec.append(v1[i] - v2[i])

    return new_vec


def axis_rot(phi, orig_vec, pos):
    """
    given angles theta and phi, this wil rotate the 
    robot about the axis for which its claw is in
    """
    # recalibrate position
    x, y, z, roll, pitch, yaw = pos




    radius = 162 # radius of rotation about roll
    #orig_vec = [0, 0, -radius]
    rot_vec = y_rot(orig_vec, phi)

    print(orig_vec)
    print(rot_vec)

    offset = sub(orig_vec, rot_vec)


    x += offset[0]
    z += offset[2]
    roll -= phi
    pos[0] += offset[0]
    pos[2] += offset[2]
    pos[3] -= phi

    arm.set_position(x=x, y = y, z = z, roll=roll, pitch=pitch, yaw=yaw, is_radian = False)
    return rot_vec
def capture_averaged_image(cap, n_avg):

    for indx in range(n_avg):
        ret, frame = cap.read()
        if not ret:
            return None
        if indx==0:
            frames=frame.astype(np.float32)
        else:
            frames = frames + frame.astype(np.float32)
    

    avg_frame = frames/n_avg
    return avg_frame

def save_frame_as_numpy(save_folder, frame, frame_count):

        # Create the filename with frame number
        filename = os.path.join(save_folder, f'image_{frame_count:04d}.npy')  # Saves as frame_0000.npy, frame_0001.npy, ...

        # Save the current frame as a NumPy array to a file
        np.save(filename, frame)

def save_frame_as_matfile(save_folder, frame, frame_count,current_angle):

        # Create the filename with frame number
        filename = os.path.join(save_folder, f'negphi2_image_{frame_count:04d}.mat')  # Saves as frame_0000.npy, frame_0001.npy, ...

        # Save the current frame as a NumPy array to a file
        savemat(filename, {'frame': frame[:,:,2],'current_angle':current_angle})

def axis_rot(phi, orig_vec, pos):
    """
    given angles theta and phi, this wil rotate the 
    robot about the axis for which its claw is in
    """
    # recalibrate position
    x, y, z, roll, pitch, yaw = pos




    radius = 162 # radius of rotation about roll
    orig_vec = [0, 0, -radius]
    rot_vec = y_rot(orig_vec, phi)

    offset = sub(orig_vec, rot_vec)


    x += offset[0]
    z += offset[2]
    roll -= phi
    pos[0] += offset[0]
    pos[2] += offset[2]
    pos[3] -= phi

    arm.set_position(x=x, y = y, z = z, roll=roll, pitch=pitch, yaw=yaw, is_radian = False)
    return rot_vec


def axis_rotation(position):
    """
    Rotates the robot arm about an arbitrary axis

    Input: 
    Position [list]; the initial position of the robot arm
    """

    print(arm.get_position()[1])
    x, y, z, roll, pitch, yaw = position

    roll, pitch = -180, 0
    if x >= 0 : # robot constantly tangles itself; fix later
        yaw = -90
    else:
        yaw = 90


    radius = 162


    arm.set_position(x=x, y = y, z = z, roll=roll, pitch=pitch, yaw=yaw, is_radian = False)




if __name__ == '__main__':
    # save_folder = "hene_laser_WP_neg2"

    # npy_file = np.load(f"{save_folder}/hene_frame_0392.npy")
    # plt.imshow(npy_file.astype(np.uint8), interpolation='nearest')
    # plt.show()
    # cv2.waitKey(2000)
    # #print(np.shape(np.mean(npy_file,axis=2)))

    # if not os.path.exists(save_folder):
    #     os.makedirs(save_folder)





    position = [-270.05249, 39.072674, 352.885986, -180, 0, -90]
    x, y, z, roll, pitch, yaw = position

    #arm.set_position(x=x, y = y, z = z, roll=roll, pitch=pitch, yaw=yaw, is_radian = False)

    axis_rotation(arm.get_position()[1])



    raise SystemExit

    cap2 = cv2.VideoCapture(2)

    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 8000)  
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 6000) 
    cap2.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)


    actual_width = int(cap2.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap2.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera resolution: {actual_width}x{actual_height}")
    frame = capture_averaged_image(cap2,100)
    save_frame_as_matfile(save_folder, frame, 481,0)

    raise SystemExit
    position = [312.920776, -390.415833, 261.360229, 179.965986, 0.004985, -90.015852]
    position = [312.920776-4, -390.415833, 261.360229-4, 179.965986, 0.004985, -90.015852]

    x, y, z, roll, pitch, yaw = position

    arm.set_position(x=x, y = y, z = z, roll=roll, pitch=pitch, yaw=yaw, is_radian = False)
    time.sleep(5)
    #raise SystemExit
    radius = 162 # radius of rotation about roll
    orig_vec = [0, 0, -radius]
    max_angle = -120
    initial_angle = -60
    angle_step = -0.25
    frame_count = 241
    current_angle = initial_angle 
    for _ in range(int(np.abs(max_angle-initial_angle)/np.abs(angle_step))):
        frame = capture_averaged_image(cap2,50)
        save_frame_as_matfile(save_folder, frame, frame_count,current_angle)

        orig_vec = axis_rot(angle_step, orig_vec, position)
        time.sleep(3)
        current_angle += angle_step
        frame_count += 1

    pass