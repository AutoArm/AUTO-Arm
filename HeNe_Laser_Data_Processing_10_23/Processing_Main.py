import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PIL import Image
import sys
from Processing import Processing
from Processing import plot_M_squared
import threading
from tqdm import tqdm
import time
import os

class ProgressBarThread:
    def __init__(self, total):
        self.total = total
        self.progress = 0
        self.done = False
        self.start_time = time.time()  # Record the start time

    def run(self):
        while not self.done:
            percent = (self.progress / self.total) * 100
            bar_length = 40  # Length of the progress bar
            block = int(bar_length * percent / 100)
            elapsed_time = time.time() - self.start_time
            
            # Calculate frames per second
            fps = self.progress / elapsed_time if elapsed_time > 0 else 0
            
            progress_bar = '█' * block + '-' * (bar_length - block)
            print(f"\rProcessing frames: |{progress_bar}| {percent:.2f}% ({self.progress}/{self.total}) | Frames per second: {fps:.2f}", end="")
            time.sleep(0.1)  # Adjust for the refresh rate

        print("\nProgress bar finished.")


def save_spherical_images():
    # Initial file pattern and parameters
    filename_after_651 = ['hene_laser/hene_frame_', '.npy']
    filename_before_650 = ['D:/hene_laser/hene_frame_', '.npy']
    n = 752  # Number of frames
    # Process frames
    save_dir = 'C:/Users/szist/Documents/GitHub/AUTO-Arm/HeNe_Laser_Data_Processing_10_23/Partial_Saves'
    os.makedirs(save_dir, exist_ok=True)

     # Progress Bar
    progress_thread = ProgressBarThread(total=n)
    thread = threading.Thread(target=progress_thread.run)
    thread.start()
    for i in range(0, n, 20):
        frame = None

        # File path selection
        if i < 650:
            file_path = filename_before_650[0] + "0" * (4 - len(str(i+1))) + str(i+1) + filename_before_650[1]
        else:
            file_path = filename_after_651[0] + "0" * (4 - len(str(i+1))) + str(i+1) + filename_after_651[1]
        
        try:
            frame = np.load(file_path)
            frame_save_path = os.path.join(save_dir, f'frame_{i+1}.npy')
            np.save(frame_save_path, frame)
            print(f"\nSaved frame {i+1} at: {frame_save_path}")

        except Exception as e:
            print(f"Error loading {file_path}: {e}")
            continue

    progress_thread.done = True
    thread.join()

    # Save the final data
    final_save_path = os.path.join(save_dir, 'Spherical_Lense_Data_Final.npz')
    np.savez(final_save_path, X=X, Y=Y, phi_array=phi_array, width_array=width_array,
             sigma_x2_array=sigma_x2_array, sigma_y2_array=sigma_y2_array, sigma_xy_array=sigma_xy_array)
    print(f"\nFinished Saving Final Data to: {final_save_path}")

def spherical_hene_laser():
    # Your initial file pattern and parameters
    filename_after_651 = ['hene_laser/hene_frame_', '.npy']
    filename_before_650 = ['D:/hene_laser/hene_frame_', '.npy']
    pixel_size = 0.00001 * 480.0 / 6000.0 # m per pixel 
    
    # Frame and letter ranges
    n = 752  # Number of frames
    X = np.ndarray((n,), dtype=np.float32)
    Y = np.ndarray((n,), dtype=np.float32)
    sigma_x2_array = np.ndarray((n,), dtype=np.float32)
    sigma_y2_array = np.ndarray((n,), dtype=np.float32)
    sigma_xy_array = np.ndarray((n,), dtype=np.float32)
    phi_array = np.ndarray((n,), dtype = np.float32)
    width_array = np.ndarray((n,), dtype=np.float32)  # Beam width array

    image_height = 6000  # Replace with actual height
    image_width = 8000   # Replace with actual width

    # Progress Bar
    progress_thread = ProgressBarThread(total=n)
    thread = threading.Thread(target=progress_thread.run)
    thread.start()

    for i in range(n):
        # Initialize accumulator for averaging the images
        frame = None
        
        if (i <= 649):
            file_path = filename_before_650[0] + "0" * (4 - len(str(i+1))) + str(i+1) + filename_before_650[1]
        else:
            file_path = filename_after_651[0] + "0" * (4 - len(str(i+1))) + str(i+1) + filename_after_651[1]
        
        try:
            #print(f"Processing frame {i + 1}")
            frame = np.load(file_path)
            
            # Find the laser center in the averaged frame
            d_sigma_x_m, d_sigma_y_m, phi, cX_m, cY_m, sigma_x2, sigma_y2, sigma_xy = Processing.compute_beam_parameters(frame, 
                                                                                            max_iterations=20, 
                                                                                            convergence_threshold=0.01, 
                                                                                            pixel_size=pixel_size)
            beam_width_m = np.sqrt((d_sigma_x_m**2 + d_sigma_y_m**2) * 0.5)  # Calculate beam width from d_sigma_x_

            # Store the averaged values in the arrays (still in m for X, Z, Width)
            X[i-1], Y[i-1] = cX_m, cY_m
            width_array[i-1] = beam_width_m
            phi_array[i-1] = np.float32(phi)
            sigma_x2_array[i-1] = sigma_x2
            sigma_y2_array[i-1] = sigma_y2
            sigma_xy_array[i-1] = sigma_xy

            progress_thread.progress += 1
        
        except Exception as e:
            print(f"Error loading {file_path}: {e}")
            continue  # Skip to the next iteration


    progress_thread.done = True  # Mark as done
    thread.join()  # Wait for progress thread to finish

    # Save to a .npz archive
    np.savez('HeNe_Laser_Data_Processing_10_23/Spherical_Lense_Data.npz', 
             X=X, 
             Y=Y, 
             phi_array = phi_array, 
             width_array = width_array,
             sigma_x2_array = sigma_x2_array,
             sigma_y2_array = sigma_y2_array,
             sigma_xy_array = sigma_xy_array,)


    print("Finished Saving")

def cylinder_x_hene_laser():
    # Your initial file pattern and parameters
    filename_pattern = ['hene_laser_cylinder_x/image_', '.npy']
    pixel_size = 0.00001 * 480.0 / 6000.0 # m per pixel 
    
    # Frame and letter ranges
    n = 1000 # Number of frames
    X = np.ndarray((n,), dtype=np.float32)
    Y = np.ndarray((n,), dtype=np.float32)
    sigma_x2_array = np.ndarray((n,), dtype=np.float32)
    sigma_y2_array = np.ndarray((n,), dtype=np.float32)
    sigma_xy_array = np.ndarray((n,), dtype=np.float32)
    phi_array = np.ndarray((n,), dtype = np.float32)
    width_array = np.ndarray((n,), dtype=np.float32)  # Beam width array

    image_height = 6000  # Replace with actual height
    image_width = 8000   # Replace with actual width

    # Progress Bar
    progress_thread = ProgressBarThread(total=n)
    thread = threading.Thread(target=progress_thread.run)
    thread.start()

    for i in range(n):
        # Initialize accumulator for averaging the images
        frame = None
        file_path = filename_pattern[0] + "0" * (4 - len(str(i+1))) + str(i+1) + filename_pattern[1]

        try:
            #print(f"Processing frame {i + 1}")
            frame = np.load(file_path)
            
            # Find the laser center in the averaged frame
            d_sigma_x_m, d_sigma_y_m, phi, cX_m, cY_m, sigma_x2, sigma_y2, sigma_xy = Processing.compute_beam_parameters(frame, 
                                                                                            max_iterations=20, 
                                                                                            convergence_threshold=0.01, 
                                                                                            pixel_size=pixel_size)
            beam_width_m = np.sqrt((d_sigma_x_m**2 + d_sigma_y_m**2) * 0.5)  # Calculate beam width from d_sigma_x_

            # Store the averaged values in the arrays (still in m for X, Z, Width)
            X[i-1], Y[i-1] = cX_m, cY_m
            width_array[i-1] = beam_width_m
            phi_array[i-1] = np.float32(phi)
            sigma_x2_array[i-1] = sigma_x2
            sigma_y2_array[i-1] = sigma_y2
            sigma_xy_array[i-1] = sigma_xy

            progress_thread.progress += 1
        except Exception as e:
            print(f"Error loading {file_path}: {e}")
            continue  # Skip to the next iteration


    progress_thread.done = True  # Mark as done
    thread.join()  # Wait for progress thread to finish

    # Save to a .npz archive
    np.savez('HeNe_Laser_Data_Processing_10_23/Cylinder_x_Lense_Data.npz', 
             X=X, 
             Y=Y, 
             phi_array = phi_array, 
             width_array = width_array,
             sigma_x2_array = sigma_x2_array,
             sigma_y2_array = sigma_y2_array,
             sigma_xy_array = sigma_xy_array,)

    print("Finished Saving")

def cylinder_y_hene_laser():
    # Your initial file pattern and parameters
    filename_pattern = ['hene_laser_cylinder_y/image_', '.npy']
    pixel_size = 0.00001 * 480.0 / 6000.0 # m per pixel 
    
    # Frame and letter ranges
    n = 1000 # Number of frames
    X = np.ndarray((n,), dtype=np.float32)
    Y = np.ndarray((n,), dtype=np.float32)
    sigma_x2_array = np.ndarray((n,), dtype=np.float32)
    sigma_y2_array = np.ndarray((n,), dtype=np.float32)
    sigma_xy_array = np.ndarray((n,), dtype=np.float32)
    phi_array = np.ndarray((n,), dtype = np.float32)
    width_array = np.ndarray((n,), dtype=np.float32)  # Beam width array

    image_height = 6000  # Replace with actual height
    image_width = 8000   # Replace with actual width

    # Progress Bar
    progress_thread = ProgressBarThread(total=n)
    thread = threading.Thread(target=progress_thread.run)
    thread.start()

    for i in range(n):
        # Initialize accumulator for averaging the images
        frame = None
        file_path = filename_pattern[0] + "0" * (4 - len(str(i+1))) + str(i+1) + filename_pattern[1]

        try:
            #print(f"Processing frame {i + 1}")
            frame = np.load(file_path)
            
            # Find the laser center in the averaged frame
            d_sigma_x_m, d_sigma_y_m, phi, cX_m, cY_m, sigma_x2, sigma_y2, sigma_xy = Processing.compute_beam_parameters(frame, 
                                                                                            max_iterations=20, 
                                                                                            convergence_threshold=0.01, 
                                                                                            pixel_size=pixel_size)
            beam_width_m = np.sqrt((d_sigma_x_m**2 + d_sigma_y_m**2) * 0.5)  # Calculate beam width from d_sigma_x_

            # Store the averaged values in the arrays (still in m for X, Z, Width)
            X[i-1], Y[i-1] = cX_m, cY_m
            width_array[i-1] = beam_width_m
            phi_array[i-1] = np.float32(phi)
            sigma_x2_array[i-1] = sigma_x2
            sigma_y2_array[i-1] = sigma_y2
            sigma_xy_array[i-1] = sigma_xy

            progress_thread.progress += 1
        except Exception as e:
            print(f"Error loading {file_path}: {e}")
            continue  # Skip to the next iteration


    progress_thread.done = True  # Mark as done
    thread.join()  # Wait for progress thread to finish

    # Save to a .npz archive
    np.savez('HeNe_Laser_Data_Processing_10_23/Cylinder_y_Lense_Data.npz', 
             X=X, 
             Y=Y, 
             phi_array = phi_array, 
             width_array = width_array,
             sigma_x2_array = sigma_x2_array,
             sigma_y2_array = sigma_y2_array,
             sigma_xy_array = sigma_xy_array,)
    

def plot_data():
    spher_coor = [452.804047, 346.466125, 248.699631] 
    # y -= 0.5  x +=0  z += .022
     
    spherical_values = [np.linspace(spher_coor[0], spher_coor[0], 752),
                        np.linspace(spher_coor[2], spher_coor[2] + 0.22*752, 752),
                        np.linspace(spher_coor[1], spher_coor[1] + 0.5*752, 752)]
    Z_values = spherical_values[2]

    cylinder_x_values = np.array((3, 1000))
    cylinder_y_values = np.array((3, 1000))
    cylinder_coor = [452.803528, 339.844727, 253.982056]

    spherical = np.load("HeNe_Laser_Data_Processing_10_23/Spherical_Lense_Data.npz")
    
    X = spherical["X"] + spherical_values[0]
    Y = spherical["Y"] + spherical_values[1]
    width_array = spherical["width_array"]


    # 3D Plot for X, Y vs Z
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Linear Regression on X and Y vs Z 
    A = np.vstack([Z_values, np.ones(len(Z_values),)]).T
    A_regression = np.linalg.inv(A.T @ A) @ A.T 
    coef = A_regression @ np.vstack([X.T, Y.T]).T
    m_x, m_y, c_x, c_y = coef[0, 0], coef[0, 1], coef[1, 0], coef[1, 1]

    # Calculate residuals
    X_residuals = X - (m_x * Z_values + c_x)
    Y_residuals = Y - (m_y * Z_values + c_y)

    # Calculate standard error for X and Z
    SE_X = np.sqrt(np.sum(X_residuals**2) / (len(Z_values) - 2))  # Standard error of X
    SE_Z = np.sqrt(np.sum(Y_residuals**2) / (len(Z_values) - 2))  # Standard error of Z

    # Output the standard errors
    print(f"Standard Error of X: {SE_X:.5f}")
    print(f"Standard Error of Z: {SE_Z:.5f}")

    # Average standard error
    average_SE = (SE_X + SE_Z) / 2
    print(f"Average Standard Error: {average_SE:.5f}")

    # Plot the 3D line for X, Z vs Y
    ax.plot(X, Y, Z_values, label='3D Line')
    ax.set_xlabel('X axis (m)')
    ax.set_ylabel('Y axis (m)')
    ax.set_zlabel('Z axis (m)')

    # Plot the regression line
    ax.plot(m_x * Z_values + c_x, m_y * Z_values + c_y, Z_values, color='red', label='Regression Line')

    # Show the legend
    ax.legend()

    # Display the 3D plot
    plt.show()

    wavelength = 633e-9 # 633nm
    plot_M_squared(Z_values[::20], width_array[::20], wavelength)



if __name__ == '__main__':
    #spherical_hene_laser()
    #cylinder_x_hene_laser()
    #cylinder_y_hene_laser()
    #spherical_hene_laser()

    save_spherical_images()

    plot_data()
    

    spherical = np.load("HeNe_Laser_Data_Processing_10_23/Spherical_Lense_Data.npz")
    cy_x = np.load("HeNe_Laser_Data_Processing_10_23/Cylinder_x_Lense_Data.npz")
    cy_y = np.load("HeNe_Laser_Data_Processing_10_23/Cylinder_y_Lense_Data.npz") 
   
    

    # for spherical lens camera_coor=  [452.804047, 346.466125, 248.699631, -179.127004, 2.657493, -91.79586] 
    # "nene_laser"                      y -= 0.5    x +=0         z += .022
    # for cylindrica lens vertical camera_coor=  [452.803528, 339.844727, 253.982056, -179.127119, 2.657607, -91.795975]         
    #      "hene_laser_cylinder_x"      y -= 1*0.5  x -= 1*0.006  z -= 1*.006
    #      "hene_laser_cylinder_y"      y -= 1*0.5  x -= 1*0.007  z -= 1*0.00
    # for key in spherical.files:
    #    print(f"Array Name: {key}")
    #    print(spherical[key].shape)

    # for key in cy_x.files:
    #    print(f"Array Name: {key}")
    #    print(cy_x[key].shape)

    # for key in cy_y.files:
    #    print(f"Array Name: {key}")
    #    print(cy_y[key].shape) 