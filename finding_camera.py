import cv2
import numpy as np
import pyrealsense2 as rs

# Test Camera 1 with MSMF
print(cv2.__version__)

cap0 = cv2.VideoCapture(0, cv2.CAP_MSMF)
cap5 = cv2.VideoCapture(5, cv2.CAP_MSMF)
cap1 = cv2.VideoCapture(1, cv2.CAP_MSMF)
cap2 = cv2.VideoCapture(2, cv2.CAP_MSMF)
cap3 = cv2.VideoCapture(3, cv2.CAP_MSMF)

# pipeline = rs.pipeline()
# config = rs.config()

# cap2 = cv2.VideoCapture(1)  # For Windows MSMF
while True:
    ret0, frame0 = cap0.read()
    ret1, frame1 = cap1.read()
    ret5, frame5 = cap5.read()
    ret2, frame2 = cap2.read()
    ret3, frame3 = cap3.read()
    ret2, frame2 = cap2.read()
    cv2.imshow('Camera 0', frame0)
    cv2.imshow('Camera 1', frame1)
    cv2.imshow('Camera 5', frame5)
    cv2.imshow('Camera 2', frame2)
    cv2.imshow('Camera 3', frame3)
    cv2.imshow('Camera 2', frame2)

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
