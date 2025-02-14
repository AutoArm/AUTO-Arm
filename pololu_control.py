import serial
from sys import version_info
import cv2
import scipy.io as sio
import time
import os
import pyfirmata
import time
import numpy as np
from scipy.ndimage import median_filter
from scipy.ndimage import label


PY2 = version_info[0] == 2   #Running Python 2.x?

# put pololu control in USB dual mode
#---------------------------
# Maestro Servo Controller
#---------------------------
#
# Support for the Pololu Maestro line of servo controllers
#
# Steven Jacobs -- Aug 2013
# https://github.com/FRC4564/Maestro/
#
# These functions provide access to many of the Maestro's capabilities using the
# Pololu serial protocol
#
class Controller:
    # When connected via USB, the Maestro creates two virtual serial ports
    # /dev/ttyACM0 for commands and /dev/ttyACM1 for communications.
    # Be sure the Maestro is configured for "USB Dual Port" serial mode.
    # "USB Chained Mode" may work as well, but hasn't been tested.
    #
    # Pololu protocol allows for multiple Maestros to be connected to a single
    # serial port. Each connected device is then indexed by number.
    # This device number defaults to 0x0C (or 12 in decimal), which this module
    # assumes.  If two or more controllers are connected to different serial
    # ports, or you are using a Windows OS, you can provide the tty port.  For
    # example, '/dev/ttyACM2' or for Windows, something like 'COM3'.
    def __init__(self,ttyStr='COM4',device=0x0c):
        # Open the command port
        self.usb = serial.Serial(ttyStr)
        # Command lead-in and device number are sent for each Pololu serial command.
        self.PololuCmd = chr(0xaa) + chr(device)
        # Track target position for each servo. The function isMoving() will
        # use the Target vs Current servo position to determine if movement is
        # occuring.  Upto 24 servos on a Maestro, (0-23). Targets start at 0.
        self.Targets = [0] * 24
        # Servo minimum and maximum targets can be restricted to protect components.
        self.Mins = [0] * 24
        self.Maxs = [0] * 24
        
    # Cleanup by closing USB serial port
    def close(self):
        self.usb.close()

    # Send a Pololu command out the serial port
    def sendCmd(self, cmd):
        cmdStr = self.PololuCmd + cmd
        if PY2:
            self.usb.write(cmdStr)
        else:
            self.usb.write(bytes(cmdStr,'latin-1'))

    # Set channels min and max value range.  Use this as a safety to protect
    # from accidentally moving outside known safe parameters. A setting of 0
    # allows unrestricted movement.
    #
    # ***Note that the Maestro itself is configured to limit the range of servo travel
    # which has precedence over these values.  Use the Maestro Control Center to configure
    # ranges that are saved to the controller.  Use setRange for software controllable ranges.
    def setRange(self, chan, min, max):
        self.Mins[chan] = min
        self.Maxs[chan] = max

    # Return Minimum channel range value
    def getMin(self, chan):
        return self.Mins[chan]

    # Return Maximum channel range value
    def getMax(self, chan):
        return self.Maxs[chan]
        
    # Set channel to a specified target value.  Servo will begin moving based
    # on Speed and Acceleration parameters previously set.
    # Target values will be constrained within Min and Max range, if set.
    # For servos, target represents the pulse width in of quarter-microseconds
    # Servo center is at 1500 microseconds, or 6000 quarter-microseconds
    # Typcially valid servo range is 3000 to 9000 quarter-microseconds
    # If channel is configured for digital output, values < 6000 = Low ouput
    def setTarget(self, chan, target):
        # if Min is defined and Target is below, force to Min
        if self.Mins[chan] > 0 and target < self.Mins[chan]:
            target = self.Mins[chan]
        # if Max is defined and Target is above, force to Max
        if self.Maxs[chan] > 0 and target > self.Maxs[chan]:
            target = self.Maxs[chan]
        #    
        lsb = target & 0x7f #7 bits for least significant byte
        msb = (target >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        cmd = chr(0x04) + chr(chan) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)
        # Record Target value
        self.Targets[chan] = target
        
    # Set speed of channel
    # Speed is measured as 0.25microseconds/10milliseconds
    # For the standard 1ms pulse width change to move a servo between extremes, a speed
    # of 1 will take 1 minute, and a speed of 60 would take 1 second.
    # Speed of 0 is unrestricted.
    def setSpeed(self, chan, speed):
        lsb = speed & 0x7f #7 bits for least significant byte
        msb = (speed >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        cmd = chr(0x07) + chr(chan) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)

    # Set acceleration of channel
    # This provide soft starts and finishes when servo moves to target position.
    # Valid values are from 0 to 255. 0=unrestricted, 1 is slowest start.
    # A value of 1 will take the servo about 3s to move between 1ms to 2ms range.
    def setAccel(self, chan, accel):
        lsb = accel & 0x7f #7 bits for least significant byte
        msb = (accel >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        cmd = chr(0x09) + chr(chan) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)
    
    # Get the current position of the device on the specified channel
    # The result is returned in a measure of quarter-microseconds, which mirrors
    # the Target parameter of setTarget.
    # This is not reading the true servo position, but the last target position sent
    # to the servo. If the Speed is set to below the top speed of the servo, then
    # the position result will align well with the acutal servo position, assuming
    # it is not stalled or slowed.
    def getPosition(self, chan):
        cmd = chr(0x10) + chr(chan)
        self.sendCmd(cmd)
        lsb = ord(self.usb.read())
        msb = ord(self.usb.read())
        return (msb << 8) + lsb

    # Test to see if a servo has reached the set target position.  This only provides
    # useful results if the Speed parameter is set slower than the maximum speed of
    # the servo.  Servo range must be defined first using setRange. See setRange comment.
    #
    # ***Note if target position goes outside of Maestro's allowable range for the
    # channel, then the target can never be reached, so it will appear to always be
    # moving to the target.  
    def isMoving(self, chan):
        if self.Targets[chan] > 0:
            if self.getPosition(chan) != self.Targets[chan]:
                return True
        return False
    
    # Have all servo outputs reached their targets? This is useful only if Speed and/or
    # Acceleration have been set on one or more of the channels. Returns True or False.
    # Not available with Micro Maestro.
    def getMovingState(self):
        cmd = chr(0x13)
        self.sendCmd(cmd)
        if self.usb.read() == chr(0):
            return False
        else:
            return True

    # Run a Maestro Script subroutine in the currently active script. Scripts can
    # have multiple subroutines, which get numbered sequentially from 0 on up. Code your
    # Maestro subroutine to either infinitely loop, or just end (return is not valid).
    def runScriptSub(self, subNumber):
        cmd = chr(0x27) + chr(subNumber)
        # can pass a param with command 0x28
        # cmd = chr(0x28) + chr(subNumber) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)

    # Stop the current Maestro Script
    def stopScript(self):
        cmd = chr(0x24)
        self.sendCmd(cmd)

def capture_averaged_image(cap, n_avg):
    frames = []
    for _ in range(n_avg):
        ret, frame = cap.read()
        if not ret:
            return None
        frames.append(frame)
    
    # Convert to float32 for averaging
    frames = [frame.astype(np.float16) for frame in frames]
    
    # Compute the average
    avg_frame = np.mean(frames, axis=0).astype(np.uint8)
    
    return avg_frame










# def process_image(image):
#     """
#     Convert RGB image to grayscale and find coordinates of maximum intensity
#     relative to image center.
    
#     Args:
#         image: numpy array of shape (height, width, 3) containing RGB image
        
#     Returns:
#         tuple containing:
#             - grayscale_image: numpy array of shape (height, width)
#             - relative_coords: tuple (y, x) of coordinates relative to image center
#     """
#     # Input validation
#     if not isinstance(image, np.ndarray) or image.ndim != 3 or image.shape[2] != 3:
#         raise ValueError("Input must be a 3D numpy array with shape (height, width, 3)")
    
#     # Convert to grayscale using luminosity method
#     # Weights based on human perception: R: 0.299, G: 0.587, B: 0.114
#     grayscale_image = np.dot(image, [0.299, 0.587, 0.114])
    
#     # Find coordinates of maximum intensity
#     max_y, max_x = np.unravel_index(np.argmax(grayscale_image), grayscale_image.shape)
    
#     # Calculate image center
#     center_y = grayscale_image.shape[0] // 2
#     center_x = grayscale_image.shape[1] // 2
    
#     # Calculate coordinates relative to center
#     relative_y = max_y - center_y
#     relative_x = max_x - center_x
    
#     return relative_y, relative_x


# def find_laser_spot(img_array):
#     """
#     Find the coordinates of a laser spot in a color image array relative to the center.
    
#     Args:
#         img_array (numpy.ndarray): Image array of shape (height, width, 3) in float16
        
#     Returns:
#         tuple: (x, y) coordinates relative to center, where:
#             - x: horizontal distance from center (positive is right)
#             - y: vertical distance from center (positive is up)
#     """
    
#     # Find maximum intensity across all color channels
#     max_intensity = np.max(img_array, axis=2)
    
#     # Find coordinates of maximum intensity
#     y_max, x_max = np.unravel_index(np.argmax(max_intensity), max_intensity.shape)
    
#     # Calculate center coordinates
#     center_x = max_intensity.shape[1] // 2  # 8000 // 2 = 4000
#     center_y = max_intensity.shape[0] // 2  # 6000 // 2 = 3000
    
#     # Calculate relative coordinates (flip y-axis so positive is up)
#     x_rel = x_max - center_x
#     y_rel = -(y_max - center_y)  # Negative because image coordinates increase downward
    
#     return x_rel, y_rel

def step_up_motor(servo,which_motor,how_many_steps,puase_time=0.2,step_time=0.05):
    servo.setTarget(which_motor,6000)  #set servo to move to stop position
    time.sleep(puase_time)
    for _ in range(int(how_many_steps)):
        servo.setTarget(which_motor,6200)  #set servo to move to ccw position
        time.sleep(step_time)
        servo.setTarget(which_motor,6000)  #set servo to move to stop position
        time.sleep(puase_time)

def step_down_motor(servo,which_motor,how_many_steps,puase_time=0.2,step_time=0.05):
    servo.setTarget(which_motor,6000)  #set servo to move to stop position
    time.sleep(puase_time)
    for _ in range(int(how_many_steps)):
        servo.setTarget(which_motor,5700)  #set servo to move to ccw position
        time.sleep(step_time)
        servo.setTarget(which_motor,6000)  #set servo to move to stop position
        time.sleep(puase_time)

def stop_motor(servo,which_motor,puase_time=0.2):
    servo.setTarget(which_motor,6000)  #set servo to move to stop position
    time.sleep(puase_time)

def color_to_grayscale(img):
    return np.dot(img[..., :3], [0.2989, 0.5870, 0.1140])

def max_location_from_center(arr):
    y, x = np.unravel_index(arr.argmax(), arr.shape)
    return x - arr.shape[1]//2, y - arr.shape[0]//2

def filter_and_threshold(arr, kernel_size=3, threshold=225):
    threshold=.90*arr.max()
    filtered = median_filter(arr, size=kernel_size)
    return filtered > threshold

def detect_laser_spot(image, kernel_size=3,threshold=225):
    arr_r=color_to_grayscale(image)
    arr=filter_and_threshold(arr_r,kernel_size,threshold)
    cframe = get_largest_cluster(arr,10)
    y, x = max_location_from_center(cframe)
    return y, x

def get_largest_cluster(matrix, threshold_percent=10):
    # Create binary mask of significant values
    mask = matrix > (matrix.max() * threshold_percent/100)
    
    # Label connected components
    labeled_matrix, _ = label(mask)
    
    # Find label of largest cluster
    label_sizes = np.bincount(labeled_matrix.ravel())[1:]
    largest_label = np.argmax(label_sizes) + 1
    
    # Create mask of largest cluster and apply to original
    largest_mask = labeled_matrix == largest_label
    result = matrix * largest_mask
    
    return result

servo = Controller()
servo.setAccel(0,0)      #set servo 0 acceleration to 0
servo.setSpeed(0,0)     #set speed of servo 0
servo.setAccel(1,0)      #set servo 0 acceleration to 0
servo.setSpeed(1,0)     #set speed of servo 0

stop_motor(servo,0)
stop_motor(servo,1)

#step_up_motor(servo,0,10)
#step_down_motor(servo,0,10)


save_path = r"C:\Users\szist\Desktop\Beam_steering"  # Replace with your desired save location
n_avg = 1 # Number of frames to average for each save


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



from matplotlib import pyplot as plt
plt.imshow(avg_frame, interpolation='nearest')
plt.show()
cv2.waitKey(2000)
print(np.shape(avg_frame))

# arr_g=color_to_grayscale(avg_frame)
# gray_frame=filter_and_threshold(arr_g)
# cframe = get_largest_cluster(gray_frame,10)

# from matplotlib import pyplot as plt
# plt.imshow(cframe, interpolation='nearest')
# plt.show()
# cv2.waitKey(2000)
# print(np.shape(cframe))


# x,y=detect_laser_spot(avg_frame)
# print(f" Pos : ({x}, {y})")

# raise SystemExit
iteration_count=0
steps=5
puase_time=0.3
step_time=0.07
cutoff_distance=100
current_distance=cutoff_distance+1

while iteration_count<100 and current_distance>cutoff_distance:
    for which_motor in range(2):
        avg_frame = capture_averaged_image(cap3, n_avg)
        xc, yc = detect_laser_spot(avg_frame)
        #print(f"Laser spot coordinates relative to center: ({xc}, {yc})")
        print(np.sqrt(xc*xc+yc*yc))
        current_distance=np.sqrt(xc*xc+yc*yc)
        #print(f"Raw pixel coordinates: ({xc + 4000}, {3000 - yc})")

        # from matplotlib import pyplot as plt
        # plt.imshow(avg_frame, interpolation='nearest')
        # plt.show()
        # cv2.waitKey(1)
    
        step_up_motor(servo,which_motor,steps,puase_time,step_time)
        avg_frame = capture_averaged_image(cap3, n_avg)
        x, y = detect_laser_spot(avg_frame)
        #print(f"Laser spot coordinates relative to center after Servo {which_motor} moves: ({x}, {y})")
        current_distance_servo1=np.sqrt(x*x+y*y)
        jump_factor=2
        if current_distance_servo1>current_distance:
            step_down_motor(servo,which_motor,jump_factor*steps,puase_time,step_time)
        else:
            step_up_motor(servo,which_motor,(jump_factor-1)*steps,puase_time,step_time)
        
        # from matplotlib import pyplot as plt
        # plt.imshow(avg_frame, interpolation='nearest')
        # plt.show()
        # cv2.waitKey(1)


    iteration_count+=1
















# if avg_frame is not None:
#     # Save the averaged image in MAT format
#     file_name = os.path.join(save_path, f"image_{indx+1}.mat")
#     sio.savemat(file_name, {'image': avg_frame})
#     print(f"Saved averaged image {file_name} (from {n_avg} frames)")
# else:
#     print(f"Error: Could not capture averaged frame {indx+1}")


# servo.setTarget(0,6000)  #set servo to move to stop position
# time.sleep(0.2)
# servo.setTarget(1,6000)  #set servo to move to stop position
# time.sleep(0.2)

# servo.setTarget(0,6200)  #set servo to move to ccw position
# time.sleep(2.05)

# servo.setTarget(0,6000)  #set servo to move to center position
# time.sleep(0.5)

# servo.setTarget(0,5700)  #set servo to move to ccw position
# time.sleep(2.05)

# servo.setTarget(0,6000)  #set servo to move to center position
# time.sleep(0.5)

# x = servo.getPosition(0) #get the current position of servo 1











# current_error=1000
# last_distance=np.sqrt(x*x+y*y)
# iteration_count=0
# while current_error>5 and iteration_count<100:
#     avg_frame = capture_averaged_image(cap3, n_avg)
#     x, y = find_laser_spot(avg_frame)
#     current_distance=np.sqrt(x*x+y*y)
#     print(f"Current distance : ({current_distance})")

#     servo.setTarget(0,6000)  #set servo to move to stop position
#     time.sleep(0.5)
#     servo.setTarget(0,6200)  #set servo to move to ccw position
#     time.sleep(0.05)
#     servo.setTarget(0,6000)  #set servo to move to stop position
#     time.sleep(0.5)
#     avg_frame = capture_averaged_image(cap3, n_avg)
#     x, y = find_laser_spot(avg_frame)
#     current_distance_servo1=np.sqrt(x*x+y*y)
#     if current_distance_servo1>current_distance:
#         print(current_distance_servo1-current_distance)
#         servo.setTarget(0,5700)  #set servo to move to cw position
#         time.sleep(0.05+0.05*(current_distance_servo1-current_distance))
#         servo.setTarget(0,6000)  #set servo to move to stop position
#         time.sleep(0.5)
#         avg_frame = capture_averaged_image(cap3, n_avg)
#         x, y = find_laser_spot(avg_frame)
#         current_distance=np.sqrt(x*x+y*y)
#     else:
#         current_distance=current_distance_servo1
#     print(f"Current distance : ({current_distance})")

#     servo.setTarget(1,6000)  #set servo to move to stop position
#     time.sleep(0.5)
#     servo.setTarget(1,6200)  #set servo to move to ccw position
#     time.sleep(0.05)
#     servo.setTarget(1,6000)  #set servo to move to center position
#     time.sleep(0.5)
#     avg_frame = capture_averaged_image(cap3, n_avg)
#     x, y = find_laser_spot(avg_frame)
#     current_distance_servo2=np.sqrt(x*x+y*y)
#     if current_distance_servo2>current_distance:
#         print(current_distance_servo2-current_distance)
#         servo.setTarget(1,5700)  #set servo to move to cw position
#         time.sleep(0.05+.05*(current_distance_servo2-current_distance))
#         servo.setTarget(1,6000)  #set servo to move to stop position
#         time.sleep(0.5)
#         avg_frame = capture_averaged_image(cap3, n_avg)
#         x, y = find_laser_spot(avg_frame)
#         current_distance=np.sqrt(x*x+y*y)
#     else:
#         current_distance=current_distance_servo2
#     print(f"Current distance : ({current_distance})")
#     current_error = np.abs(current_distance-last_distance)
#     last_distance=current_distance
#     iteration_count+=1


# current_error=1000
# last_distance=np.sqrt(x*x+y*y)
# iteration_count=0
# distance_list=[]
# while current_error>5 and iteration_count<100:
#     distance_list = np.append(distance_list, last_distance) 
#     avg_frame = capture_averaged_image(cap3, n_avg)
#     x, y = find_laser_spot(avg_frame)
#     current_distance=np.sqrt(x*x+y*y)
#     print(f"Current distance : ({current_distance})")

#     servo.setTarget(0,6000)  #set servo to move to stop position
#     time.sleep(0.5)
#     servo.setTarget(0,6200)  #set servo to move to ccw position
#     time.sleep(0.05)
#     servo.setTarget(0,6000)  #set servo to move to stop position
#     time.sleep(0.5)
#     avg_frame = capture_averaged_image(cap3, n_avg)
#     x, y = find_laser_spot(avg_frame)
#     current_distance_servo1=np.sqrt(x*x+y*y)
#     if current_distance_servo1>current_distance:
#         print(current_distance_servo1-current_distance)
#         servo.setTarget(0,5700)  #set servo to move to cw position
#         time.sleep(0.05+0.01*(current_distance_servo1-current_distance))
#         servo.setTarget(0,6000)  #set servo to move to stop position
#         time.sleep(0.5)
#         avg_frame = capture_averaged_image(cap3, n_avg)
#         x, y = find_laser_spot(avg_frame)
#         current_distance=np.sqrt(x*x+y*y)
#     else:
#         print(current_distance_servo1-current_distance)
#         servo.setTarget(0,6200)  #set servo to move to cw position
#         time.sleep(0.05+0.01*(-current_distance_servo1+current_distance))
#         servo.setTarget(0,6000)  #set servo to move to stop position
#         time.sleep(0.5)
#         avg_frame = capture_averaged_image(cap3, n_avg)
#         x, y = find_laser_spot(avg_frame)
#         current_distance=np.sqrt(x*x+y*y)
#     print(f"Current distance servo 1 : ({current_distance})")

#     servo.setTarget(1,6000)  #set servo to move to stop position
#     time.sleep(0.5)
#     servo.setTarget(1,6200)  #set servo to move to ccw position
#     time.sleep(0.05)
#     servo.setTarget(1,6000)  #set servo to move to center position
#     time.sleep(0.5)
#     avg_frame = capture_averaged_image(cap3, n_avg)
#     x, y = find_laser_spot(avg_frame)
#     current_distance_servo2=np.sqrt(x*x+y*y)
#     if current_distance_servo2>current_distance:
#         print(current_distance_servo2-current_distance)
#         servo.setTarget(1,5700)  #set servo to move to cw position
#         time.sleep(0.05+.01*(current_distance_servo2-current_distance))
#         servo.setTarget(1,6000)  #set servo to move to stop position
#         time.sleep(0.5)
#         avg_frame = capture_averaged_image(cap3, n_avg)
#         x, y = find_laser_spot(avg_frame)
#         current_distance=np.sqrt(x*x+y*y)
#     else:
#         print(current_distance_servo2-current_distance)
#         servo.setTarget(1,6200)  #set servo to move to cw position
#         time.sleep(0.05+.01*(-current_distance_servo2+current_distance))
#         servo.setTarget(1,6000)  #set servo to move to stop position
#         time.sleep(0.5)
#         avg_frame = capture_averaged_image(cap3, n_avg)
#         x, y = find_laser_spot(avg_frame)
#         current_distance=np.sqrt(x*x+y*y)
#     print(f"Current distance servo 2 : ({current_distance})")
#     current_error = np.abs(current_distance-last_distance)
#     last_distance=current_distance
      
#     iteration_count+=1

# iteration_count=0
# while iteration_count<4:
#     xc, yc = find_laser_spot(avg_frame)
#     print(f"Laser spot coordinates relative to center: ({xc}, {yc})")
#     print(np.sqrt(xc*xc+yc*yc))
#     current_distance=np.sqrt(xc*xc+yc*yc)
#     print(f"Raw pixel coordinates: ({xc + 4000}, {3000 - yc})")

#     dt=1.0
#     servo.setTarget(0,6000)  #set servo to move to stop position
#     time.sleep(0.2)
#     servo.setTarget(0,6200)  #set servo to move to ccw position
#     time.sleep(dt)
#     servo.setTarget(0,6000)  #set servo to move to stop position
#     time.sleep(0.2)
#     avg_frame = capture_averaged_image(cap3, n_avg)
#     x, y = find_laser_spot(avg_frame)
#     print(f"Laser spot coordinates relative to center after Servo 1 moves: ({x}, {y})")
#     servo.setTarget(0,5700)  #set servo to move to cw position
#     time.sleep(dt)
#     servo.setTarget(0,6000)  #set servo to move to stop position
#     time.sleep(0.2)

#     current_distance_servo1=np.sqrt(x*x+y*y)
#     time_on_servo1 = (0-y)/((y-yc)/(dt))
#     print(time_on_servo1)
#     if time_on_servo1>0:
#         servo.setTarget(0,6000)  #set servo to move to stop position
#         time.sleep(0.2)
#         servo.setTarget(0,6200)  #set servo to move to ccw position
#         time.sleep(time_on_servo1/10)
#         servo.setTarget(0,6000)  #set servo to move to stop position
#         time.sleep(0.2)
#     elif time_on_servo1<0:
#         servo.setTarget(0,6000)  #set servo to move to stop position
#         time.sleep(0.2)
#         servo.setTarget(0,5700)  #set servo to move to ccw position
#         time.sleep(-time_on_servo1/10)
#         servo.setTarget(0,6000)  #set servo to move to stop position
#         time.sleep(0.2)


#     avg_frame = capture_averaged_image(cap3, n_avg)
#     xc, yc = find_laser_spot(avg_frame)
#     current_distance=np.sqrt(xc*xc+yc*yc)

#     servo.setTarget(1,6000)  #set servo to move to stop position
#     time.sleep(0.2)
#     servo.setTarget(1,6200)  #set servo to move to ccw position
#     time.sleep(dt)
#     servo.setTarget(1,6000)  #set servo to move to stop position
#     time.sleep(0.2)
#     avg_frame = capture_averaged_image(cap3, n_avg)
#     x, y = find_laser_spot(avg_frame)
#     current_distance_servo2=np.sqrt(x*x+y*y)
#     servo.setTarget(1,5700)  #set servo to move to ccw position
#     time.sleep(dt)
#     servo.setTarget(1,6000)  #set servo to move to stop position
#     time.sleep(0.2)

#     time_on_servo2 = (0-x)/((x-xc)/(dt))
#     print(time_on_servo2)
#     if time_on_servo2>0:
#         servo.setTarget(1,6000)  #set servo to move to stop position
#         time.sleep(0.2)
#         servo.setTarget(1,6200)  #set servo to move to ccw position
#         time.sleep(time_on_servo2/10)
#         servo.setTarget(1,6000)  #set servo to move to stop position
#         time.sleep(0.2)
#     else:
#         servo.setTarget(1,6000)  #set servo to move to stop position
#         time.sleep(0.2)
#         servo.setTarget(1,5700)  #set servo to move to ccw position
#         time.sleep(-time_on_servo2/10)
#         servo.setTarget(1,6000)  #set servo to move to stop position
#         time.sleep(0.2)
#     iteration_count+=1





# Release the camera
cap3.release()
servo.close()
