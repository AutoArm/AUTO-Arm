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

def rotate(plane, theta):
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


def vector_to_euler(vector):
    """
    Convert a 3D vector to roll, pitch, yaw angles in degrees.
    Known cases:
    [0,0,-1] -> roll=-180°, pitch=0°, yaw=-90°
    [0.70711, 0, -0.70711] -> roll=-145°, pitch=0°, yaw=-90°
    [-0.70711, 0, -0.70711] -> roll=145°, pitch=0°, yaw=-90°
    [-0.0002, -0.7069, -0.7073] -> roll=180°, pitch=-45°, yaw=-90°
    [-4.52e-05, 0.7071, -0.7071] -> roll=180°, pitch=45°, yaw=-90°
    
    Args:
        vector: numpy array of shape (3,) representing direction vector
        
    Returns:
        tuple: (roll, pitch, yaw) angles in degrees
    """
    # Normalize the vector
    vector = vector / np.linalg.norm(vector)
    x, y, z = vector
    
    if np.allclose([x, y, z], [0, 0, -1]):
        return -180.0, 0.0, -90.0
    
    # Calculate yaw (always close to -90 for these cases)
    yaw = -90.0
    
    # Calculate pitch - angle from XY plane
    # For vectors with significant Y component
    if abs(y) > 0.1:
        pitch = np.degrees(np.arctan2(z, -y))
    else:
        pitch = 0.0
    
    # Calculate roll
    if abs(y) > 0.1:
        roll = 180.0
    else:
        roll = 180.0 - np.degrees(np.arccos(-z))
        if x > 0:
            roll = -roll
    
    return roll, pitch, yaw



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
    
    # Project end effector direction onto plane perpendicular to rotation axis
    # This ensures we rotate in the correct plane
    proj = curr_end_effector - np.dot(curr_end_effector, plane) * plane
    proj = normalize(proj)
    
    # Create rotation matrix for phi rotation around plane vector
    phi_rad = np.radians(phi)
    cos_phi = np.cos(phi_rad)
    sin_phi = np.sin(phi_rad)
    
    # Get perpendicular vector to form basis for rotation
    perp = normalize(np.cross(plane, proj))
    
    # Calculate new end effector direction using the rotation in the plane
    new_end_effector = proj * cos_phi + perp * sin_phi + \
                      plane * np.dot(curr_end_effector, plane)
    new_end_effector = normalize(new_end_effector)
    
    # Convert new direction to roll, pitch, yaw
    new_roll, new_pitch, new_yaw = vector_to_euler(new_end_effector)
    
    # Calculate new base position to maintain end point position
    new_base_pos = curr_end_point - new_end_effector * radius
    
    # Move to new position with new orientation
    arm.set_position(
        x=new_base_pos[0],
        y=new_base_pos[1],
        z=new_base_pos[2],
        roll=new_roll,
        pitch=new_pitch,
        yaw=new_yaw,
        is_radian=False
    )

from scipy.spatial.transform import Rotation
def rotate_phi(axis, phi, radius=200):
    """
    Rotates the robot arm around an arbitrary axis vector by phi degrees,
    maintaining the laser contact point while rotating the end effector.
    
    Args:
        axis (np.array): Axis to rotate around (e.g. [0,-1,0] for lens normal)
        phi (float): Rotation angle in degrees
        radius (float): Distance from base position to end effector
    """
    # Get current position
    position = arm.get_position()
    x, y, z, curr_roll, curr_pitch, curr_yaw = position[1]
    curr_pos = np.array([x, y, z])
    
    # Get current end effector direction
    curr_end_effector = get_end_effector_direction(curr_roll, curr_pitch, curr_yaw)
    
    # Normalize vectors
    axis = normalize(np.array(axis))
    curr_end_effector = normalize(curr_end_effector)
    
    # Calculate current end point (where laser hits lens)
    curr_end_point = curr_pos + curr_end_effector * radius
    
    # Convert angle to radians
    phi_rad = np.radians(phi)
    
    # Rotate end effector direction using Rodrigues formula
    cos_phi = np.cos(phi_rad)
    sin_phi = np.sin(phi_rad)
    
    new_end_effector = (curr_end_effector * cos_phi + 
                       np.cross(axis, curr_end_effector) * sin_phi + 
                       axis * np.dot(axis, curr_end_effector) * (1 - cos_phi))
    new_end_effector = normalize(new_end_effector)


    print(new_end_effector)
    
    # Calculate new base position to maintain end point
    new_base_pos = curr_end_point - new_end_effector * radius
    
    # Convert new direction to roll, pitch, yaw
    new_pitch = -np.arcsin(new_end_effector[2])
    new_roll = np.arctan2(new_end_effector[1], new_end_effector[0])
    new_yaw = 0  # We may need to adjust this
    
    # Set new position and orientation
    arm.set_position(
        x=new_base_pos[0],
        y=new_base_pos[1],
        z=new_base_pos[2],
        roll=np.degrees(new_roll),
        pitch=np.degrees(new_pitch),
        yaw=new_yaw,
        is_radian=False
    )

if __name__ == '__main__':


    plane = [0,-1,0]

    new_plane = rotate(plane, 0) # update radius of face_shift if center moves


    # rotate_phi(new_plane,0)




    #arm.set_position(x=x, y = y, z = z, roll=roll, pitch=pitch, yaw=yaw, is_radian = False)

