import cv2
import numpy as np
import pyrealsense2 as rs
'''
Date: Fall 2024
Description: Use code to find which cameras are on which ports
'''
# Test Camera 1 with MSMF
print(cv2.__version__)

cap3 = cv2.VideoCapture(4, cv2.CAP_MSMF)
cap3.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap3.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
# pipeline = rs.pipeline()
# config = rs.config()

# cap2 = cv2.VideoCapture(1)  # For Windows MSMF
while True:
    ret1, frame1 = cap3.read()
    # ret2, frame2 = cap2.read()
    cv2.imshow('Camera 1', frame1)
    # cv2.imshow('Camera 2', frame2)

    # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    # config.enable_stream(rs.stream.depth,640,480,rs.format.z16,15)

    # pipeline.start(config)
    
    # frames = pipeline.wait_for_frames()
    # depth_frame = frames.get_depth_frame()
    # depth_image = np.asanyarray(depth_frame.get_data())
    # color_frame = frames.get_color_frame()
    # color_image = np.asanyarray(color_frame.get_data())
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
