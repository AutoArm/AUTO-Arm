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
from scipy.spatial.transform import Rotation as R



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


def vector_to_rpy(vector):
    """
    Convert a direction vector to roll, pitch, yaw angles.
    
    Args:
        vector (np.array): 3D vector [vx, vy, vz] that we want to point to
        
    Returns:
        tuple: (roll, pitch, yaw) in degrees
    """

    # Normalize the vector
    vector = np.array(vector, dtype=float)
    vector = vector / np.linalg.norm(vector)
    vx, vy, vz = vector
    
    # pitch (rot abt y-axis)
    pitch = np.degrees(np.arctan2(-vz, np.sqrt(vx*vx + vy*vy)))
    
    # yaw (rotation abt z-axis)
    yaw = np.degrees(np.arctan2(vy, vx))
    
    # calc roll
    if np.isclose(vz, 1.0):  # pointing straight up
        roll = 0
    elif np.isclose(vz, -1.0):  # pointing straight down
        roll = 180
    else:
        roll = 180 if vy < 0 else 0
    
    return roll, pitch, yaw




def normalize(v):
    """Normalize a vector"""
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

def get_end_effector_direction(roll_deg, pitch_deg, yaw_deg):
    """
    Calculate the direction vector of the end effector from roll, pitch, and yaw angles in degrees.
    For a robot arm, this represents the direction the hand/gripper is pointing.
    When the arm points straight down, this should be [0, 0, -1].
    """
    # Convert degrees to radians
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    yaw = np.deg2rad(yaw_deg)
    
    # Create rotation matrices
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Combine rotations
    R = Rz @ Ry @ Rx  # Try ZYX order
    
    # The end effector direction is typically aligned with one of the principal axes
    direction = -R[:, 2]  # Using negative Z column as default
    direction = normalize(direction)

    
    
    return -1*direction


def rotate_vector(vector, axis, angle_degrees):
    """
    Rotate a vector around an arbitrary axis by a given angle using Rodrigues' rotation formula.
    
    Parameters:
    vector (array-like): The vector to be rotated (3D)
    axis (array-like): The axis of rotation (3D)
    angle_degrees (float): The rotation angle in degrees
    
    Returns:
    numpy.ndarray: The rotated vector
    """
    # Convert inputs to numpy arrays and normalize the rotation axis
    v = np.array(vector, dtype=float)
    k = np.array(axis, dtype=float)
    k = k / np.linalg.norm(k)
    v = v / np.linalg.norm(v)
    
    # Convert angle to radians
    theta = np.radians(angle_degrees)
    
    # Rodrigues' rotation formula
    # v_rot = v*cos(θ) + (k×v)*sin(θ) + k*(k·v)*(1-cos(θ))
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    
    # Calculate the three terms of Rodrigues' formula
    term1 = v * cos_theta
    term2 = np.cross(k, v) * sin_theta
    term3 = k * np.dot(k, v) * (1 - cos_theta)
    
    # Combine terms to get the rotated vector
    v_rotated = term1 + term2 + term3
    v_rotated = normalize(v_rotated)
    
    return v_rotated




def face(plane):
    """
    given a vector (plane) that is normal to the plane 
    of the optical component, this will move the robot arm to that position
    """

    # TODO: add functionality to keep center the same
    plane = np.array(plane, dtype = float)
    plane = normalize(plane)

    position = arm.get_position()
    x, y, z, roll, pitch, yaw = position[1]
    new_roll, new_pitch, new_yaw = vector_to_rpy(plane)

    arm.set_position(x=x, y=y, z=z, roll=new_roll, pitch=new_pitch, yaw=new_yaw, is_radian=False)

def face_shift(plane, radius = 200):
    """
    makes the plane of the robot face a direction that 
    is normal to the plane 

    radius is distance from x,y,z to end effector
    """

    plane = np.array(plane, dtype = float)
    plane = normalize(plane)

    position = arm.get_position()
    x, y, z, roll, pitch, yaw = position[1]
    end_eff = get_end_effector_direction(roll, pitch, yaw) * radius

    # current x, y, and z of end effector
    cx, cy, cz = end_eff[0] + x, end_eff[1] + y, end_eff[2] + z

    # calculate new x, y, and z of end effector 
    new_roll, new_pitch, new_yaw = vector_to_rpy(plane)
    new_end_eff = get_end_effector_direction(new_roll, new_pitch, new_yaw) * radius

    nx, ny, nz = new_end_eff[0] + x, new_end_eff[1] + y, new_end_eff[2] + z
    dx, dy, dz = nx - cx, ny - cy, nz - cz

    print(dx, dy, dz)

    arm.set_position(x=x-dx, y=y-dy, z=z-dz, roll=new_roll, pitch=new_pitch, yaw=new_yaw, is_radian=False)

def rotate_theta(plane, theta):
    """
    assumes already facing the plane

    given a vector normal to the robot's plane (plane)
    this func will make the robot move to that point 
    rotated abt theta

    note: follows right hand rule with end effector pointing towards hand
    """
    plane = np.array(plane, dtype = float)
    plane = normalize(plane)

    roll, pitch, yaw = vector_to_rpy(plane)

    end_effector = get_end_effector_direction(roll, pitch, yaw) # rotate 
    end_effector = normalize(end_effector)

    axis = np.cross(end_effector, plane)


    new_plane = rotate_vector(plane, axis, theta)

    face_shift(new_plane)

    return new_plane



def vector_to_unit(v):
    v = np.array(v, dtype=float)
    norm = np.linalg.norm(v)
    if norm < 1e-15:
        raise ValueError("Zero-length vector.")
    return v / norm

def axis_angle_to_quaternion(axis, angle):
    # Axis is a unit vector
    half_angle = angle / 2.0
    s = math.sin(half_angle)
    w = math.cos(half_angle)
    x = axis[0]*s
    y = axis[1]*s
    z = axis[2]*s
    return np.array([w, x, y, z], dtype=float)

def quaternion_to_rotation_matrix(q):
    w, x, y, z = q
    # Rotation matrix from quaternion
    # R = [[1-2y²-2z²,  2xy-2wz,    2xz+2wy ],
    #      [2xy+2wz,    1-2x²-2z²,  2yz-2wx ],
    #      [2xz-2wy,    2yz+2wx,    1-2x²-2y²]]
    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),       1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y),       2*(y*z + w*x),       1 - 2*(x**2 + y**2)]
    ], dtype=float)
    return R

def align_vectors(v1, v2):
    """
    Given two 3D vectors v1 and v2, returns (roll, pitch, yaw) in degrees, where:
    - roll is rotation about Y-axis
    - pitch is rotation about X-axis
    - yaw is rotation about Z-axis
    using the intrinsic rotation sequence: yaw (Z), then pitch (X), then roll (Y).
    """
    # Normalize
    v1_u = vector_to_unit(v1)
    v2_u = vector_to_unit(v2)

    # Check if they are almost identical
    dot_val = np.dot(v1_u, v2_u)
    if abs(dot_val - 1.0) < 1e-12:
        return 0.0, 0.0, 0.0

    # If opposite
    if abs(dot_val + 1.0) < 1e-12:
        # Rotate 180 degrees about any axis perpendicular to v1
        perp = np.array([1,0,0], dtype=float)
        if abs(np.dot(perp, v1_u)) > 0.99:
            perp = np.array([0,1,0], dtype=float)
        axis = np.cross(v1_u, perp)
        axis /= np.linalg.norm(axis)
        angle = math.pi
    else:
        # General case
        axis = np.cross(v1_u, v2_u)
        axis /= np.linalg.norm(axis)
        angle = math.acos(dot_val)

    # Axis-angle to quaternion
    q = axis_angle_to_quaternion(axis, angle)

    # Quaternion to rotation matrix
    R = quaternion_to_rotation_matrix(q)

    # Extract Euler angles from rotation matrix
    # Given R = R_z(yaw)*R_x(pitch)*R_y(roll), we have:
    # θ = pitch = asin(R[2,1])
    # ψ = yaw   = atan2(-R[0,1], R[1,1])
    # φ = roll  = atan2(-R[2,0], R[2,2])
    #
    # roll = φ (about Y), pitch = θ (about X), yaw = ψ (about Z)
   
    # Check for numerical issues
    # Clamp R[2,1] within [-1,1] before asin
    val = R[2,1]
    val = max(min(val,1.0), -1.0)
   
    pitch = math.asin(val)    # θ
    yaw = math.atan2(-R[0,1], R[1,1])  # ψ
    roll = math.atan2(-R[2,0], R[2,2]) # φ

    # Convert to degrees
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)

    return (roll_deg, pitch_deg, yaw_deg)


def rotate_phi(plane, phi, radius=200):
    """
    Rotates the robot arm around the plane normal vector by phi degrees
    while maintaining the end effector's absolute position in space.
    
    Args:
        plane (list/array): Vector normal to the plane [x, y, z]
        phi (float): Rotation angle about the plane normal (degrees)
        radius (float): Distance from base position to end effector
    """
    # Normalize plane vector
    plane = np.array(plane, dtype=float)
    plane = normalize(plane)
    
    # Get current position
    position = arm.get_position()
    x, y, z, curr_roll, curr_pitch, curr_yaw = position[1]
    
    # Get the current end effector direction
    curr_end_effector = get_end_effector_direction(curr_roll, curr_pitch, curr_yaw)
    
    # Calculate the current end point in space
    curr_end_point = np.array([x, y, z]) + curr_end_effector * radius

    goal = rotate_vector(curr_end_effector, plane, phi) # target for where the end effector should end up


    print(list(map(float,curr_end_effector)))
    print(list(map(float, goal)))
    droll, dyaw, dpitch = align_vectors(curr_end_effector, goal)

    print(droll, dpitch, dyaw)

    droll, dpitch, dyaw = -55, -30, 55

    arm.set_position(x=x, y=y, z=z, 
                     roll=curr_roll+droll, pitch=curr_pitch+dpitch, yaw = curr_yaw+dyaw)

    

if __name__ == '__main__':


    plane = [0,-1,0]

    #new_plane = rotate_theta(plane, 45) # update radius of face_shift if center moves

    #time.sleep(5)

    #rotate_phi(new_plane, 90)

    #print(align_vectors([0,0,-1], [1,0,0]))

    x, y, z, roll, pitch, yaw = arm.get_position()[1]

    rotate_theta(plane, 0)

    # print(get_end_effector_direction(roll, pitch, yaw))

    #arm.set_position(x=x, y = y, z = z, roll=roll, pitch=pitch, yaw=yaw, is_radian = False)


