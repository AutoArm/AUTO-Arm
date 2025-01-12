<img width="900" alt="image" src="https://github.com/user-attachments/assets/158b2e49-fe04-482e-b284-c9e55afadde4">

#### The AUTO Arm is an effort by the MIT Theoretical Physics Lab to create a robot that can assemble complex optical configurations quickly, speeding up the process by 90 percent. Furthermore, the robot can also implement AI and ML techniques to optimize layouts and to make second harmonic generation a possibility.
## Specifications
 - XArm 7
 - Stereo vision using two 4k Cameras: [ELP 4K USB Camera](https://www.amazon.com/ELP-Microphone-5-50mm-Varifocal-Vari-focus/dp/B0BVFKTM6Z/ref=asc_df_B0BVFKTM6Z/?tag=hyprod-20&linkCode=df0&hvadid=693308325592&hvpos=&hvnetw=g&hvrand=53687256479189848&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9191806&hvtargid=pla-2088917187303&psc=1&mcid=d2580302f178300ebafadc199fc36bd2&gad_source=1)
 - RealSense Depth Camera D435
 - Windows 10 development with Python 3.10

## Overview
 - The system utilizes **Aruco** Tags to autonomously find the position of elements placed in the bank.
 - The Stereo System finds the approximate location of the tag
 - The Arm moves to the predicted position placing RealSense camera above the tag
 - The Realsense camera fine adjusts and finds depth and orientation using linear algebra
 - Finally, we can pick up the object with 0.1mm translational precision and a tenth of a degree rotational precision

## Code
### Current work is on in_lab branch, old milestone on main branch
- Camera and Image processing
   - [Processing.py](https://github.com/AutoArm/AUTO-Arm/blob/in_lab/Camera%20and%20Image%20Processing/Processing.py): Code for the processing of data (Author: )
   - [create_aruco_tag.py](https://github.com/AutoArm/AUTO-Arm/blob/in_lab/Camera%20and%20Image%20Processing/create_aruco_tag.py): Code to create Aruco Tag with ID and size (Author: [Shrish](https://github.com/ShrishChou))
   - [finding_camera.py](https://github.com/AutoArm/AUTO-Arm/blob/in_lab/Camera%20and%20Image%20Processing/finding_camera.py): Use code to find which cameras are on which ports (Author: [Shrish](https://github.com/ShrishChou))
   - [read_camera_with_full_resolution.py](https://github.com/AutoArm/AUTO-Arm/blob/in_lab/Camera%20and%20Image%20Processing/Processing.py): Figuring out resolution issues with camera (Author: )
   - [thorcam.py](https://github.com/AutoArm/AUTO-Arm/blob/in_lab/Camera%20and%20Image%20Processing/thorcam.py): Code for using special camera (Author: [Shrish](https://github.com/ShrishChou))

- Experiments
   - [FAT.py](https://github.com/AutoArm/AUTO-Arm/blob/in_lab/Experiments/FAT.py): FAT code that used an approach based on changes to the overall computer vision **NOT USED IN DEMO STILL IN WORKINGS** (Author: [Shrish](https://github.com/ShrishChou))
   - [FAT_old.py](https://github.com/AutoArm/AUTO-Arm/blob/in_lab/Experiments/FAT_old.py): Demo version of the FAT used for our final demo (Author: [Shrish](https://github.com/ShrishChou))
   - [Measurement_pipeline.py](https://github.com/AutoArm/AUTO-Arm/blob/in_lab/Experiments/Measurement_pipeline.py): Code for finding the ports where the cameras are located. Change port values based on availability (varies per machine) (Author: [Shrish](https://github.com/ShrishChou))
   - [axis_rotation.py](https://github.com/AutoArm/AUTO-Arm/blob/in_lab/Experiments/axis_rotation.py): Rotate the arm around an axis (Author: )
   - [beam_alignment.py](https://github.com/AutoArm/AUTO-Arm/blob/in_lab/Experiments/beam_aligment.py): Given the general area where the laser is, the robot uses the camera to center the robot and moves back. Used in our paper for finding beam parameters (Author: [Shrish](https://github.com/ShrishChou))
   - [interferometer.py](https://github.com/AutoArm/AUTO-Arm/blob/in_lab/Experiments/interferometer.py): Paper work for the creation of the interferometer (Author: [Shrish](https://github.com/ShrishChou))
   - [rotate gripper 12-12.py](https://github.com/AutoArm/AUTO-Arm/blob/in_lab/Experiments/rotate%20gripper%2012-12.py): (Author: )
   - [video_demo.py](https://github.com/AutoArm/AUTO-Arm/blob/in_lab/Experiments/video_demo.py): First Interferometer setup video demo code (Author: [Shrish](https://github.com/ShrishChou))
 
- Miscellaneous
   - All extra files for different tests

- Testing
   - [alignment.py](https://github.com/ShrishChou/AUTO-Arm/blob/in_lab/Testing/alginment.py): Testing the capabilities of the robot and try to check the repeatability and precision of the dropping (Author: [Shrish](https://github.com/ShrishChou))
   - [aligment_with_drop.py](https://github.com/ShrishChou/AUTO-Arm/blob/in_lab/Testing/aligment_with_drop.py): Code to do the testing of the abilities of the robot but this time with the dropping (Author: [Shrish](https://github.com/ShrishChou))
   - [fine_adjustment.py](https://github.com/ShrishChou/AUTO-Arm/blob/in_lab/Testing/fine_adjustment.py): Code for the implementation of the Lidar camera (Author: [Shrish](https://github.com/ShrishChou))
   - [fulladjustment.py](https://github.com/ShrishChou/AUTO-Arm/blob/in_lab/Testing/fulladjustment.py): Code for the implementation of the complete CV system (Author: [Shrish](https://github.com/ShrishChou))
   - [internaltest.py](https://github.com/ShrishChou/AUTO-Arm/blob/in_lab/Testing/internaltest.py): Code to check if the detection system is working (Author: [Shrish](https://github.com/ShrishChou))

```python

# Code that users Should use at high level
start() # Intialize to home position
initialize_cams() # Setup all cameras and gives you list with all references
pickup_element_with_tag(cams,tag_id,size=0.062,special=False) # Implementation of all CV to pickup element with specific size and tag ID
find_position_of_tag(cams,tag_id,size=0.062) # Finding the position without picking up
drop_element_at_position(arm,pos,special=False) # Dropping the element at a designated position - special for FAT
center_robot_to_laser(cams,laz_pos) # Centering the arm when it is holding the camera to a laser
gohome() # Reset arm to home position
full_pipeline(camera_tag,lens_tag) # pipeline for finding properties of beam with complete setup of 1. finding laser 2. dropping lens in front 3. move camera back and find area of beam



# Fat specific code
rotate_motor_async(board) # FAT rotating with async drivers
rotate_motor(revolutions, direction, board,option=0,speed_rpm=60) # FAT rotating with normal drivers
rotate_coordinate_plane(angle_degrees) # rotate the coordinate plane and get unit vectors based on angle of tag
calculate_approach_vector(coor,rotation) # calculate how to approach the element (uses ^ )
move_along_vector(arm, start_pose, direction, distance=26) # approach the element with the FAT

# Code that is used in functions more lower level
move_to(arm,coor) # Move to a coordinate
center_arm(arm,frame) # Center the arm to a laser when holding a camera
pickup_claw(arm,coor,pipeline,target_id,special=False) # pickup with the claw attachement - special in the case of the FAT, pipeline is realsense
pickup_claw_stay(arm,coor,pipeline) # pickup_claw but without homing - if need to perform actions after
drop_claw(arm,coor,special) # dropping the element - special for case of FAT
find_pos(arm,coor,pipeline,target_id) # Find the position of a tag
rotate_claw(arm,rotation_angle) # for rotating the final servo only

# Vision Algebra
calculate_area(corner)
calculate_rotation_angle(corner)
```

## Using
- Run pip install requirements.txt
- Run [fulladjustment.py](https://github.com/ShrishChou/AUTO-Arm/blob/main/fulladjustment.py) and wait for the 4k cameras to boot up.
- Once the start procedure has finished and the robot has homed, place the item and make sure the cameras can see the tag.

## Issues/Warnings and Solutions
- IP Issues: When setting up IP for the computer, set alternate IP configuration to 192.168.1 with basic masks and DNS Servers. Check that packages are being recieved and sent afterwards
- Camera port Access: Ensure that many cameras are not connected at the same time, ensure the camera app is off at the time of use, and make sure to use finding_camera.py to find ports of cameras before hand
- Calibration Issues: Use chackerboard of known size and measure dimensions of boxes after print. **Specified checkerboard size may differ to print.** Adjust scaling factor by measuring actual distance between camera and predicted translational distance
- RealSense Issues: Ensure correct version of Python is downloaded (3.6~3.10) if not you must wipe the virtual environment, downgrade, and then activate a new environment
- Movement Issues: Using the given set_position will often fail. For this reason I have developed a movement code that utilizes trigonometry and planned motion to eliminate collisions. Do not use set_position and instead utilize the pickup and drop methods
- Camera Crash issues: Due to the running of 2 4k cameras and a RealSense camera, some systems cannot handle the numerous feeds. For this you can change the stream code to instead take a singular picture at the start of each motion rather than keeping continuous streams for the 4k cameras.
- Gripper VS Suction Gripper: For the use of different grippers, different methods have been made. Depending on which attachment is being used please use the correct code.
- Crashing of robot: Ensure that robot joints are not overheating and make sure the robot is enabled with sufficient power
- Detecting Failure: Ensure that correct cameras are being used and make sure that tag is in range of **both** cameras due to stereo system
- Issues in 2nd quadrant: Due to rotational constraints and issues with set_position, be sure to only let the base servo turn to a max of 90 degrees (from the horizontal) and let set_position do calculations
- Issues with horizontal movement: Cameras may or may not be reversed. In our code the horizontal is reversed so we have negated the values. Be sure to adjust the value on your cases
- **Warning**: **Do NOT let the arm crash into the ground and always be positioned near the emergency stop button in case of failure**
- **Warning**: **Be sure to not move stereo system after calibration or else readings will fail**
