import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PIL import Image
'''
Date: Fall 2024
Description: Code for the processing of data
'''
class Processing:
    def __init__(self):
        """
        Initialize the Processing class.
        """
        pass

    @staticmethod
    def check_saturation(image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        _, saturation, _ = cv2.split(hsv)
        avg_saturation = np.mean(saturation)
        saturation_threshold = 150
        if avg_saturation > saturation_threshold:
            print("Error: High saturation detected! Average saturation:", avg_saturation)

    @staticmethod
    def real_time_laser_detection():
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error: Could not open camera.")
            return

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break

            Processing.check_saturation(frame)

            laser_center = Processing.find_laser_center(frame)
            if laser_center is not None:
                cv2.circle(frame, laser_center, 10, (0, 255, 0), -1)
            else:
                print("Laser spot not detected")

            cv2.imshow("Laser Detection", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
    
    @staticmethod
    def estimate_pose(corners, mtx, dist, marker_length):
        """
        Estimate the pose of an ArUco marker.

        Parameters
        ----------
        corners : list of numpy.ndarray
            Detected corners of the ArUco marker.
        mtx : numpy.ndarray
            Camera matrix from calibration.
        dist : numpy.ndarray
            Distortion coefficients from calibration.
        marker_length : float
            Side length of the marker in meters.

        Returns
        -------
        rvec : numpy.ndarray
            Rotation vector representing the orientation of the marker.
        tvec : numpy.ndarray
            Translation vector representing the position of the marker.
        """
        obj_points = np.array([
            [-marker_length / 2, marker_length / 2, 0],
            [marker_length / 2, marker_length / 2, 0],
            [marker_length / 2, -marker_length / 2, 0],
            [-marker_length / 2, -marker_length / 2, 0]
        ])
        success, rvec, tvec = cv2.solvePnP(obj_points, corners, mtx, dist)
        return rvec, tvec

    @staticmethod
    def rotated_rectangle_mask(x_coords, y_coords, centroid_x, centroid_y, d_sigma_x, d_sigma_y, phi):
        """
        Integrate over a rotated rectangular area, aligned with the principal axes of the beam.
        """
        # Step 1: Shift the coordinates so that the centroid is at the origin
        x_shifted = x_coords - centroid_x
        y_shifted = y_coords - centroid_y

        # Step 2: Rotate the coordinates by the angle -phi (to align the rectangle with the principal axes)
        cos_phi = np.cos(-phi)
        sin_phi = np.sin(-phi)
        x_rotated = cos_phi * x_shifted - sin_phi * y_shifted
        y_rotated = sin_phi * x_shifted + cos_phi * y_shifted

        # Step 3: Define the bounds for the rectangular area (3 * beam widths along x and y)
        half_width_x = 3 * d_sigma_x / 2
        half_width_y = 3 * d_sigma_y / 2

        # Step 4: Create mask to select pixels inside the rotated rectangle
        mask = (np.abs(x_rotated) <= half_width_x) & (np.abs(y_rotated) <= half_width_y)

        return mask
    
    @staticmethod
    def rgb_to_intensity(rgb_image):
        # Convert the RGB image to floating-point values for higher precision
        rgb_image = rgb_image.astype(np.float64)

        # Apply the luminance formula for each pixel
        intensity = 0.2989 * rgb_image[:, :, 0] + 0.5870 * rgb_image[:, :, 1] + 0.1140 * rgb_image[:, :, 2]

        return intensity

    @staticmethod
    def compute_beam_parameters(image, pixel_size=0.00001, max_iterations=20, convergence_threshold=0.01):
        """
        Calculate the beam widths along the principal axes using BeamCharacterizationISO, adhering to ISO standards.
        
        Parameters
        ----------
        image : numpy.ndarray
            Image representing the beam profile.
        pixel_size : float, optional
            Pixel size in m to convert moments to physical dimensions (default is 0.00001 m/pixel).
        max_iterations : int, optional
            Maximum number of iterations for centroid refinement (default is 20).
        convergence_threshold : float, optional
            Threshold for convergence of centroid position refinement (default is 1%).

        Returns
        -------
        d_sigma_x : float
            Beam width along the x principal axis in m.
        d_sigma_y : float
            Beam width along the y principal axis in m.
        phi_z : float
            Orientation angle of the beam's principal axes in rad
        centroid_x : float
            The x-coordinate of the beam's centroid in m.
        centroid_y : float
            The y-coordinate of the beam's centroid in m.
        """
        
        contour_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray_image = Processing.rgb_to_intensity(image)

        # Flatten the intensity and create x, y coordinates
        intensity = gray_image.flatten()
        x_coords, y_coords = np.meshgrid(np.arange(gray_image.shape[1]), np.arange(gray_image.shape[0]))
        x_coords = x_coords.flatten()
        y_coords = y_coords.flatten()

        # First Estimation of centroid and beam widths
        centroid_x, centroid_y = Processing.compute_first_order_moments(intensity, x_coords, y_coords)
        sigma_x2, sigma_y2, sigma_xy = Processing.compute_second_order_moments(intensity, x_coords, y_coords, centroid_x, centroid_y)
        phi_z = Processing.compute_orientation(sigma_x2, sigma_y2, sigma_xy)
        d_sigma_x, d_sigma_y = Processing.compute_beam_width(sigma_x2, sigma_y2, sigma_xy)

        contours, hierarchy = cv2.findContours(contour_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 

        max_contour = contours[0]
        max_length = cv2.arcLength(max_contour, True)
        for contour in contours:
            if (max_length < cv2.arcLength(contour, True)):
                max_contour = contour
                max_length = cv2.arcLength(contour, True)
        
        # Calculate the distance of each pixel from the centroid
        contour_distances = np.sqrt((max_contour[:, 0, 0] - centroid_x)**2 + (max_contour[:, 0, 1] - centroid_y)**2)
        distances = np.sqrt((x_coords - centroid_x)**2 + (y_coords - centroid_y)**2)
        # Calculate the Maximum distance from centroid
        max_distance = 3*np.max(contour_distances)
        
        for iteration in range(max_iterations):
            if False:
                print("Iteration: " + str(iteration))
                x = np.arange(640)
                y = np.arange(480)
                x_plots, y_plots = np.meshgrid(x, y)

                fig = plt.figure()
                ax = fig.add_subplot(111, projection='3d')

                ax.plot_surface(x_plots, y_plots, intensity.reshape(480, 640), cmap='viridis')

                ax.set_xlabel('X Axis')
                ax.set_ylabel('Y Axis')
                ax.set_zlabel('Intensity')

                plt.show()

            if (iteration == 0):
                #Create a mask to filter out pixels that are outliers (too far from the centroid)
                mask = distances <= max_distance
                mask = mask.flatten()
            else:
                # Use rotated rectangle mask to filter pixels
                mask = Processing.rotated_rectangle_mask(x_coords, 
                                                        y_coords, 
                                                        centroid_x, 
                                                        centroid_y, 
                                                        d_sigma_x, 
                                                        d_sigma_y, 
                                                        phi_z)

            # Use the mask to select pixels within the rotated rectangle
            new_intensity = intensity[mask]
            new_x_coords = x_coords[mask]
            new_y_coords = y_coords[mask]

            # Recalculate the centroid and width based on the reduced integration area
            new_centroid_x, new_centroid_y = Processing.compute_first_order_moments(new_intensity, 
                                                                                    new_x_coords, 
                                                                                    new_y_coords)
            sigma_x2, sigma_y2, sigma_xy = Processing.compute_second_order_moments(new_intensity, 
                                                                                   new_x_coords, 
                                                                                   new_y_coords, 
                                                                                   new_centroid_x, 
                                                                                   new_centroid_y)

            phi_z = Processing.compute_orientation(sigma_x2, sigma_y2, sigma_xy)
            new_d_sigma_x, new_d_sigma_y = Processing.compute_beam_width(sigma_x2, sigma_y2, sigma_xy)
            # print("New Widths")
            # print(new_d_sigma_x, new_d_sigma_y)
            # print("Old Widths")
            # print(d_sigma_x, d_sigma_y)
            # Check for convergence
            if (np.abs(new_d_sigma_x - d_sigma_x) / d_sigma_x < convergence_threshold and
                np.abs(new_d_sigma_y - d_sigma_y) / d_sigma_y < convergence_threshold):
                break
            
            centroid_x, centroid_y = new_centroid_x, new_centroid_y
            d_sigma_x, d_sigma_y = new_d_sigma_x, new_d_sigma_y

        # Convert moments to physical dimensions
        d_sigma_x = d_sigma_x * pixel_size
        d_sigma_y = d_sigma_y * pixel_size
        centroid_x = centroid_x * pixel_size
        centroid_y = centroid_y * pixel_size

        return d_sigma_x, d_sigma_y, phi_z, centroid_x, centroid_y
    
    # Calculation of centroid using first-order moments
    @staticmethod
    def compute_first_order_moments(intensity, x_coords, y_coords):
        
        m00 = np.sum(intensity)
        m10 = np.sum(x_coords * intensity)
        m01 = np.sum(y_coords * intensity)
        
        centroid_x = m10 / m00
        centroid_y = m01 / m00
        
        return centroid_x, centroid_y
    
    # Calculate second-order moments
    @staticmethod
    def compute_second_order_moments(intensity, x_coords, y_coords, centroid_x, centroid_y):
        m00 = np.sum(intensity)
        m20 = np.sum((x_coords - centroid_x)**2 * intensity)
        m02 = np.sum((y_coords - centroid_y)**2 * intensity)
        m11 = np.sum((x_coords - centroid_x) * (y_coords - centroid_y) * intensity)
        
        sigma_x2 = m20 / m00
        sigma_y2 = m02 / m00
        sigma_xy = m11 / m00
        
        return sigma_x2, sigma_y2, sigma_xy

    # Compute beam widths and orientation
    @staticmethod
    def compute_beam_width(sigma_x2, sigma_y2, sigma_xy):
        gamma = np.sign(sigma_x2 - sigma_y2) * np.sqrt((sigma_x2 - sigma_y2)**2 + 4 * sigma_xy**2)
        d_sigma_x = 2 * np.sqrt(2) * np.sqrt((sigma_x2 + sigma_y2 + gamma)) 
        d_sigma_y = 2 * np.sqrt(2) * np.sqrt((sigma_x2 + sigma_y2 - gamma)) 
        return d_sigma_x, d_sigma_y

    # Compute the orientation of the beam's principal axes
    @staticmethod
    def compute_orientation(sigma_x2, sigma_y2, sigma_xy):
        phi_z = 0.5 * np.arctan2(2 * sigma_xy, sigma_x2 - sigma_y2)
        return phi_z

    # Compute 8/10 of the Second Order Moments of Wigner Distribution 
    @staticmethod
    def compute_wigner_second_order(sigma_x2_array, sigma_y2_array, sigma_xy_array, z_array):
        sigma_x2_coef = np.polyfit(z_array, sigma_x2_array, 2)
        sigma_y2_coef = np.polyfit(z_array, sigma_y2_array, 2)
        sigma_xy_coef = np.polyfit(z_array, sigma_xy_array, 2)

        x2_zero = sigma_x2_coef[2]
        x_thetax_zero = sigma_x2_coef[1] / 2
        thetax2_zero = sigma_x2_coef[0]

        xy_zero = sigma_xy_coef[2]
        s_zero = sigma_xy_coef[1]
        theta_xy_zero = sigma_xy_coef[0]

        y2_zero = sigma_y2_coef[2]
        y_thetay_zero = sigma_y2_coef[1] / 2
        thetay2_zero = sigma_y2_coef[0]

        P_zero = np.array([[x2_zero, xy_zero, x_thetax_zero, 0],
                           [xy_zero, y2_zero, 0, y_thetay_zero],
                           [x_thetax_zero, 0, thetax2_zero, theta_xy_zero],
                           [0, y_thetay_zero, theta_xy_zero, thetay2_zero]])
        
        rayleigh_length = np.sqrt((x2_zero + y2_zero) / (thetax2_zero + thetay2_zero) - ((x_thetax_zero + y_thetay_zero) / (thetax2_zero + thetay2_zero))**2 )
        
        return P_zero, s_zero, rayleigh_length
    
    @staticmethod
    def compute_twist_second_order(P_zero, sigma_xy_y, sigma_xy_x, s_zero, focal_length):
        delta = (sigma_xy_y - sigma_xy_x) / (2 * focal_length)
        x_thetay_zero = s_zero / 2 + delta
        y_thetax_zero = s_zero / 2 - delta
        
        P_zero[0][3] = x_thetay_zero
        P_zero[3][0] = x_thetay_zero

        P_zero[1][2] = y_thetax_zero
        P_zero[2][1] = y_thetax_zero

        return x_thetay_zero, y_thetax_zero, P_zero

    # Compute Final Beam Matrix
    @staticmethod
    def compute_beam_matrix(P_zero, distance_ref_lense, focal_length):
        L = distance_ref_lense
        f = focal_length
        S = np.array([[1 - L / f, 0, -L, 0],
                      [0, 1- L / f, 0, -L],
                      [1 / f, 0, 1, 0],
                      [0, 1/f, 0, 1]])
        
        return np.matmul(np.matmul(S, P_zero), S.T)
    
    # Compute M_squared given Final Beam Matrix
    @staticmethod 
    def compute_m_squared(P, wavelength):
        return 4 * np.pi / wavelength * (np.linalg.det(P)) ** (1/4)
    
    # Compute Twist given Final Beam Matrix
    @staticmethod 
    def compute_twist(P):
        return P[0, 3] - P[1, 2]

    # Compute Intrinsic Astigmatism given Final Beam Matrix
    @staticmethod
    def compute_intrinsic_astigmatism(P, wavelength):
        M_squared = Processing.compute_m_squared(P, wavelength)
        a_x = P[0, 0] * P[2, 2] - P[0, 2] ** 2
        a_y = P[1, 1] * P[3, 3] - P[1, 3] ** 2
        a_xy = P[0, 1] * P[3, 2] - P[0, 3] * P[1, 2]

        a = 8 * (np.pi ** 2) / wavelength * (a_x + a_y) + 2*a_xy - M_squared ** 2
        return a

    # Compute Parameters if Beam is Stigmatic 
    @staticmethod
    def compute_stigmatic_parameters(Z, width, wavelength):
        # Fit a quadratic to the beam width squared (d^2 as a function of z)
        width_squared = width ** 2
        width_coef = np.polyfit(Z, width_squared, 2)  # Fit a quadratic to w(z)^2
        
        # Coefficients of the quadratic fit: a + bz + cz^2
        a = width_coef[2]
        b = width_coef[1]
        c = width_coef[0]
        
        # Calculating the beam parameters
        z_zero = -b / (2 * c)  # Beam waist location
        d_sigma_zero = 1 / (2 * np.sqrt(c)) * np.sqrt(4 * a * c - b ** 2) # Beam Waist 
        theta_sigma = np.sqrt(c)  # Beam divergence angle 
        z_rayleigh = 1 / (2 * c) * np.sqrt(4 * a * c - b ** 2)  # Rayleigh range
        
        # Calculate M^2
        M_squared = np.pi / (8 * wavelength) * np.sqrt(4 * a * c - b ** 2)

        return M_squared, d_sigma_zero, theta_sigma, z_rayleigh, z_zero, c, b, a


def gaussian_beam_test():
    # Image dimensions (640x480)
    image_width = 640
    image_height = 480

    # Physical size of each pixel (0.01 mm/pixel), used to calculate Gaussian size in pixels
    pixel_size_m = 0.00001 

    # Number of Gaussian beam images to generate
    num_images = 100

    # Parameters of the Gaussian beam
    A = 1.0              # Peak intensity at the waist
    w0_px = 50             # Beam waist radius in pixels (smallest beam size)
    w0_m = w0_px * pixel_size_m 
    lambda_m = np.float64(532e-9)       # Wavelength in m

    # Derived Rayleigh range
    z_R = np.pi * (w0_m) ** 2 / lambda_m

    # Create grid of coordinates
    x = np.linspace(-image_width // 2, image_width // 2, image_width, dtype = np.float64)
    y = np.linspace(-image_height // 2, image_height // 2, image_height, dtype = np.float64)
    x, y = np.meshgrid(x, y)

    def gaussian_beam_intensity(x, y, z, w0_px, z_R, A=1.0):
        w_z_px = w0_px * np.sqrt(1 + (z / z_R)**2)  # Beam radius at z
        intensity = A * (w0_px / w_z_px)**2 * np.exp(-2 * (x**2 + y**2) / w_z_px**2)
        return intensity, w_z_px
    
    theoretical_widths = []
    
    def create_gaussian_beam_rgb_images(x, y, z_values, w0_px, z_R, A=1.0):
        # Find the global maximum intensity across all z-values for consistent scaling
        global_max_intensity = 0
        for z in z_values:
            intensity, w_z_px = gaussian_beam_intensity(x, y, z, w0_px, z_R, A)
            theoretical_widths.append(w_z_px * pixel_size_m)
            max_intensity = np.max(intensity)
            if max_intensity > global_max_intensity:
                global_max_intensity = max_intensity

        # Create the RGB images by normalizing intensity values
        rgb_images = []
        for z in z_values:
            intensity, _ = gaussian_beam_intensity(x, y, z, w0_px, z_R, A)
            
            # Normalize by the global maximum intensity
            intensity_normalized = intensity / global_max_intensity * 255
            
            # Convert to uint8 for each channel (Red, Green, Blue)
            intensity_gray = intensity_normalized.astype(np.uint8)
            
            # Create an RGB image by stacking the intensity for all channels
            rgb_image = np.stack([intensity_gray, intensity_gray, intensity_gray], axis=-1)  # (H, W, 3)
            
            # Append the RGB image to the list
            rgb_images.append(rgb_image)
        
        return rgb_images
    
    # From -3 Rayleigh range to +3 Rayleigh range
    z_values = np.linspace(-2*z_R, 2*z_R, num_images)  
    images = create_gaussian_beam_rgb_images(x, y, z_values, w0_px, z_R, A)

    X = np.zeros((num_images,), dtype=np.float64)
    Y = np.zeros((num_images,), dtype=np.float64)
    phi_array = np.zeros((num_images,), dtype=np.float64)
    width_array = np.zeros((num_images,), dtype=np.float64)

    # Loop through z-values to generate Gaussian beam images at each z-distance
    for i, z in enumerate(z_values):
        frame = images[i]
        # Find the laser center in the averaged frame
        d_sigma_x_m, d_sigma_y_m, phi, cX_m, cY_m = Processing.compute_beam_parameters(frame, 
                                                                                           max_iterations=0, 
                                                                                           convergence_threshold=0.01, 
                                                                                           pixel_size=pixel_size_m)
        beam_width_m = np.sqrt((d_sigma_x_m**2 + d_sigma_y_m**2) * 0.5)  # Calculate beam width from d_sigma_x_
        # Store the averaged values in the arrays (still in m for X, Z, Width)
        X[i], Y[i] = cX_m, cY_m
        width_array[i] = beam_width_m
        phi_array[i] = np.float64(phi)

        plot_frames = [5*i for i in range(21)]

        if i in plot_frames:
            # Convert the frame to RGB if it's grayscale for visualization
            if len(frame.shape) == 2:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
            else:
                frame_rgb = frame

            # Plot the image
            plt.figure()
            plt.imshow(frame_rgb)


            cX_px = cX_m / pixel_size_m
            cY_px = cY_m / pixel_size_m

            # Plot the center as a red dot (in pixel coordinates)
            # plt.scatter(cX_px, cY_px, color='red', label='Beam Center')

            # Plot the beam width as a circle (in pixel coordinates, assume beam_width is diameter)
            beam_radius_px = beam_width_m / pixel_size_m / 2
            circle = plt.Circle((cX_px, cY_px), beam_radius_px,
                                color='cyan', 
                                fill=False, 
                                linewidth=1, 
                                label='Beam Width')
            plt.gca().add_patch(circle)

            theory_circle = plt.Circle((image_width / 2, image_height / 2),
                                        theoretical_widths[i] / pixel_size_m,
                                        color='green',
                                        fill=False, linewidth=1, label='Theory Width')
            plt.gca().add_patch(theory_circle)

            error_width = ((beam_width_m / 2.0) / theoretical_widths[i] - 1.0) * 100.0
            z_cur = (i - 50) / 25
            print(f"At z =  {z_cur} * rayleight range")
  
            print("Predicted Width " + str(beam_width_m / 2))
            print("Theoretical Width " + str(theoretical_widths[i]))
            print("Percent Error " + str(error_width))
            # Title and legend
            plt.title(f'Beam Center and Width (Frame: {i}) and Error {error_width}%')

            plt.legend()
            plt.grid(False)

            # Show the plot
            #plt.show()

    # Call plotXY to plot X, Y vs Z (3D plot)
    plotXY(X, Y, z_values)

    print("Actual values")
    print("z_0: " + str(0))
    print("z_rayleigh: " + str(z_R))

    # _, _, beam_divergence_angle = Processing.compute_m_squared(Y, width, wavelength=532e-9)
    plot_M_squared(z_values, width_array, lambda_m, theoretical_widths)

    

def main_1():
    # Your initial file pattern and parameters
    filename_pattern = ['saved_frames_high_res_camera_lens/frame_hi_res_', '0001', '.npy']
    pixel_size = 0.00001  # 0.00001 m per pixel 
    
    # Frame and letter ranges
    letters = [chr(i) for i in range(ord('a'), ord('a') + 1)]  # Create list of letters 'a' to 'z'
    n = 330  # Number of frames
    X = np.ndarray((n,), dtype=np.float32)
    Z = np.ndarray((n,), dtype=np.float32)
    phi_array = np.ndarray((n,), dtype = np.float32)
    width_array = np.ndarray((n,), dtype=np.float32)  # Beam width array

    # Frames for which we want to plot the center and beam width
    plot_frames =[100, 110, 120, 130, 140, 150]

    image_height = 480  # Replace with actual height
    image_width = 640   # Replace with actual width

    # Initialize a NumPy array to store the images (with dtype=np.uint8 for grayscale images)
    images = np.zeros((n, image_height, image_width), dtype=np.uint8)

    for i in range(1, n+1):
        # Initialize accumulator for averaging the images
        avg_frame = None
        print(f"Processing frame {i}")

        # Loop through the 26 letter files (a to z) for each frame and sum the images
        for letter in letters:
            file = filename_pattern[0] + "0" * (4 - len(str(i))) + str(i) + letter + filename_pattern[2]
            frame = np.load(file)

            if avg_frame is None:
                avg_frame = np.zeros_like(frame, dtype=np.float32)  # Initialize with correct shape and type

            avg_frame += frame.astype(np.float32)  # Accumulate the image values as floats

        # Divide by 26 to get the average
        avg_frame /= len(letters)

        # Convert the averaged image back to the original type if needed (e.g., if it was an 8-bit image)
        avg_frame = avg_frame.astype(np.uint8)
        images[i-1] = cv2.cvtColor(avg_frame, cv2.COLOR_BGR2GRAY)

        # Find the laser center in the averaged frame
        d_sigma_x_m, d_sigma_y_m, phi, cX_m, cY_m = Processing.compute_beam_parameters(avg_frame, 
                                                                                           max_iterations=20, 
                                                                                           convergence_threshold=0.01, 
                                                                                           pixel_size=pixel_size)
        beam_width_m = np.sqrt((d_sigma_x_m**2 + d_sigma_y_m**2) * 0.5)  # Calculate beam width from d_sigma_x_
        
        # Convert from m to pixel coordinates
        cX_px = cX_m / pixel_size
        cY_px = cY_m / pixel_size
        beam_width_px = beam_width_m / pixel_size  # Convert beam width to pixels

        # Store the averaged values in the arrays (still in m for X, Z, Width)
        X[i-1], Z[i-1] = cX_m, cY_m
        width_array[i-1] = beam_width_m
        phi_array[i-1] = np.float32(phi)

        # If the frame is one of the specified frames, plot the image, center, and beam width
        if i in plot_frames:
            # Convert the frame to RGB if it's grayscale for visualization
            if len(avg_frame.shape) == 2:
                frame_rgb = cv2.cvtColor(avg_frame, cv2.COLOR_GRAY2RGB)
            else:
                frame_rgb = avg_frame

            # Plot the image
            plt.figure()
            plt.imshow(frame_rgb)

            # Plot the center as a red dot (in pixel coordinates)
            plt.scatter(cX_px, cY_px, color='red', label='Beam Center')

            # Plot the beam width as a circle (in pixel coordinates, assume beam_width is diameter)
            beam_radius_px = beam_width_px / 2
            circle = plt.Circle((cX_px, cY_px), beam_radius_px, color='cyan', fill=False, linewidth=1, label='Beam Width')
            plt.gca().add_patch(circle)

            # Title and legend
            plt.title(f'Beam Center and Width (Frame: {i})')
            plt.legend()
            plt.grid(False)

            # Show the plot
            plt.show()

    # Define Y axis for 3D plot and beam width plot (in m)
    deltaY = .150  # .150 m
    Y = np.linspace(0, deltaY, n)


    # Call plotXY to plot X, Z vs Y (3D plot)
    plotXY(X, Z, Y)

    # _, _, beam_divergence_angle = Processing.compute_m_squared(Y, width, wavelength=532e-9)
    plot_M_squared(Y, width_array, wavelength=532e-9)

    #Processing.plot_M_squared(Y, width, wavelength=532e-9)
    #print("Beginning Beam Matrix")
    #beam_matrix(images, cX_array, cZ_array, Y, width, orientation_vector, beam_divergence_angle)
    #print("End of Beam Matrix")

def plot_frame(frame, image_width, image_height):
    x = np.arange(image_width)
    y = np.arange(image_height)
    x_coords, y_coords = np.meshgrid(x, y)
    intensity = frame

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot_surface(x_coords, y_coords, intensity, cmap='viridis')

    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Intensity')

    plt.show()

def plot_M_squared(Z, width, wavelength, theoretical_widths):
    # Compute M², min width, and beam divergence using the compute_m_squared method
    M_squared, min_width_m, beam_divergence, z_rayleigh, z_zero, c, b, a = Processing.compute_stigmatic_parameters(Z, width, wavelength)

    # Plot data points
    plt.scatter(Z, width ** 2, label="Data points")

    # Plot the fitted quadratic curve
    Z_fit = np.linspace(min(Z), max(Z), 100)
    theoretical_widths = np.array(theoretical_widths)
    width_fit = c * Z_fit**2 + b * Z_fit + a
    plt.plot(Z_fit, width_fit, label="Fitted curve", color='r')
    plt.plot(Z_fit, 4 * theoretical_widths ** 2, label = "Theory", color = 'b')

    plt.xlabel("Z (Propagation distance)")
    plt.ylabel("Beam Width Squared (w(z)^2)")

    plt.title('M² Factor = ' + str(M_squared))
    plt.legend()
    plt.grid(True)

    # Show the plot
    plt.show()

    # Print the calculated values for reference
    print(f"M² = {M_squared:.2f}")
    print(f"Minimum Beam Width (m) = {min_width_m:.5f} m")
    print(f"Beam Divergence (radians) = {beam_divergence:.5f}")

def plotXY(X, Y, Z_values):
    # 3D Plot for X, Y vs Z
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Linear Regression on X and Y vs Z 
    A = np.vstack([Z_values, np.ones(len(Z_values),)]).T
    A_regression = np.linalg.inv(A.T @ A) @ A.T 
    coef = A_regression @ np.vstack([X.T, Y.T]).T
    m_x, m_z, c_x, c_z = coef[0, 0], coef[0, 1], coef[1, 0], coef[1, 1]

    orient_vec = np.array([m_x, 1, m_z])
    yaw = np.arctan2(orient_vec[0], orient_vec[1])
    pitch = np.arctan2(orient_vec[2], np.sqrt(orient_vec[0] ** 2 + orient_vec[1] ** 2))
    roll = np.arctan2(orient_vec[1], orient_vec[0])

    print("Laser Orientation Vector:")
    
    print("yaw, pitch, roll (rad):")
    print(yaw, pitch, roll)

    # Calculate residuals
    X_residuals = X - (m_x * Z_values + c_x)
    Z_residuals = Y - (m_z * Z_values + c_z)

    # Calculate standard error for X and Z
    SE_X = np.sqrt(np.sum(X_residuals**2) / (len(Z_values) - 2))  # Standard error of X
    SE_Z = np.sqrt(np.sum(Z_residuals**2) / (len(Z_values) - 2))  # Standard error of Z

    # Output the standard errors
    print(f"Standard Error of X: {SE_X:.5f}")
    print(f"Standard Error of Z: {SE_Z:.5f}")

    # Average standard error
    average_SE = (SE_X + SE_Z) / 2
    print(f"Average Standard Error: {average_SE:.5f}")

    # Plot the 3D line for X, Z vs Y
    ax.plot(X, Z_values, Y, label='3D Line')
    ax.set_xlabel('X axis (m)')
    ax.set_ylabel('Y axis (m)')
    ax.set_zlabel('Z axis (m)')

    # Plot the regression line
    ax.plot(m_x * Z_values + c_x, Z_values, m_z * Z_values + c_z, color='red', label='Regression Line')

    # Show the legend
    ax.legend()

    # Display the 3D plot
    plt.show()

    # Output the orientation vector
    return orient_vec
if __name__ == '__main__':
    gaussian_beam_test()
    #main_1()
    

    



