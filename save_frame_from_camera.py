#!/usr/bin/env python3
import os
import time
import cv2
import traceback
import numpy as np
from xarm import version
from xarm.wrapper import XArmAPI
from matplotlib import pyplot as plt

def save_frame_as_numpy(save_folder, frame, frame_count, theta, phi):
        # Create the filename with frame number
        filename = os.path.join(save_folder, f'image_base0_{frame_count:04d}.npy')  # Saves as frame_0000.npy, frame_0001.npy, ...
        # Save the current frame as a NumPy array to a file
        np.savez(filename, frame=frame, theta=theta, phi=phi)
        #np.save(filename, frame)


def capture_averaged_image(cap, n_avg):
    for indx in range(n_avg):
        ret, frame = cap.read()
        if not ret:
            return None
        if indx==0:
            frames=frame.astype(np.float32)
        else:
            frames = frames + frame.astype(np.float16)
    

    avg_frame = frames/n_avg
    return avg_frame

def run(navg=50):

    frame_count=10

    # data=np.load('sample_transmission_theta_phi/image_base0_0010.npy.npz')
    # plt.imshow(np.round(data['frame']).astype(int), interpolation='nearest')
    # plt.show()
    # cv2.waitKey(1000)

    # raise SystemExit

    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 8000)  
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 6000) 

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera resolution: {actual_width}x{actual_height}")

    save_folder = "sample_transmission_theta_phi"

        
    # plt.imshow(np.round(frame).astype(int), interpolation='nearest')
    # plt.show()
    # cv2.waitKey(1000)


    frame = capture_averaged_image(cap, navg)
    frame = frame[2350:3450,:,:]
    save_frame_as_numpy(save_folder, frame, frame_count,0,45)
    frame_count += 1 



if __name__ == '__main__':
    navg=50
    run(navg)