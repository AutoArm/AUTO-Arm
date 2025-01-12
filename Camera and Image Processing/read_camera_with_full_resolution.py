import cv2
import numpy as np
import scipy.io as sio
import time
import os
import pyfirmata
import time
'''
Date: Fall 2024
Description: Figuring out resolution issues with camera
'''

def capture_averaged_image(cap, n_avg):
    frames = []
    for _ in range(n_avg):
        ret, frame = cap.read()
        if not ret:
            return None
        frames.append(frame)
    
    # Convert to float32 for averaging
    frames = [frame.astype(np.float32) for frame in frames]
    
    # Compute the average
    avg_frame = np.mean(frames, axis=0).astype(np.uint8)
    
    return avg_frame



save_path = r"C:\Users\szist\Desktop\Postdoc\Beam_direction_measurements"  # Replace with your desired save location
n_avg = 5 # Number of frames to average for each save


# Initialize the camera
cap3 = cv2.VideoCapture(2)  # 0 is usually the default USB camera

# Check if the camera opened successfully
if not cap3.isOpened():
    print("Error: Could not open camera.")


# Set the camera to capture at its highest resolution
cap3.set(cv2.CAP_PROP_FRAME_WIDTH, 8000)  
cap3.set(cv2.CAP_PROP_FRAME_HEIGHT, 6000) 

# Get the actual resolution
actual_width = int(cap3.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_height = int(cap3.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Camera resolution: {actual_width}x{actual_height}")

# Create the save directory if it doesn't exist
#os.makedirs(save_path, exist_ok=True)


# Capture and average multiple frames
avg_frame = capture_averaged_image(cap3, n_avg)

# if avg_frame is not None:
#     # Save the averaged image in MAT format
#     file_name = os.path.join(save_path, f"image_{indx+1}.mat")
#     sio.savemat(file_name, {'image': avg_frame})
#     print(f"Saved averaged image {file_name} (from {n_avg} frames)")
# else:
#     print(f"Error: Could not capture averaged frame {indx+1}")

# Release the camera
cap3.release()

from matplotlib import pyplot as plt
plt.imshow(avg_frame, interpolation='nearest')
plt.show()
cv2.waitKey(2000)
print(np.shape(avg_frame))
