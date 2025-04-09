
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
from demo24 import pickup_element_with_tag
from time import sleep

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from PIL import Image
from xarm.wrapper import XArmAPI

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

arm = XArmAPI("192.168.1.241")
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
arm.set_gripper_enable(True)
code = arm.set_gripper_speed(2000)
print("init done")





if __name__ == '__main__':
    print("demo starting")

    
    
    
    
    # get initial positioning
    code,place=arm.get_position(is_radian=False)


    if code != 0:
        print("Read Position is Incorrect")
        raise SystemExit
    x, y, z, roll, pitch, yaw = place
    roll,pitch,yaw = 180,0,-90 # normalize so end effector is straight
    z = 450


    arm.set_position(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, is_radian=False)
    sleep(2)


    tx, ty, tz, troll, tpitch, tyaw = 44.3, -404.1, 299.6, 180, 0, -140

    arm.set_position(x=tx, y=ty, z=z, roll=troll, pitch = tpitch, yaw = tyaw)
    sleep(2)
    arm.set_position(x=tx, y=ty, z=tz, roll=troll, pitch = tpitch, yaw = tyaw)

    raise SystemExit

    arm.set_gripper_position(600,wait=True)


    arm.set_position(x=tx, y=ty, z=380, roll=180, pitch = 0, yaw = -90)

    sleep(4)

    arm.set_position(x=362.1, y=322.6, z=310.6, roll=180, pitch = 0, yaw = -90)

    sleep(2)

    arm.set_position(x=362.1, y=-243.6, z=310.6, roll=180, pitch = 0, yaw = -90)







    raise SystemExit


    



    