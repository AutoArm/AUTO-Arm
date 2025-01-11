import cv2
import numpy as np

class Processing:
    def __init__(self):
        """
        Initialize the Processing class.
        """
        pass
    
    @staticmethod
    def estimate_pose(self, corners, mtx, dist, marker_length):
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
    def average_rotation_vectors(self, rvecs):
        """
        Average multiple rotation vectors.

        Parameters
        ----------
        rvecs : list of numpy.ndarray
            List of rotation vectors.

        Returns
        -------
        avg_rvec : numpy.ndarray
            Averaged rotation vector.
        """
        matrices = [cv2.Rodrigues(rvec)[0] for rvec in rvecs]
        avg_matrix = sum(matrices) / len(matrices)

        # Ensure the averaged matrix is a valid rotation matrix
        U, _, Vt = np.linalg.svd(avg_matrix)
        avg_matrix_orthogonal = U @ Vt

        avg_rvec, _ = cv2.Rodrigues(avg_matrix_orthogonal)
        return avg_rvec

    @staticmethod
    def extract_euler_angles(self, rvec):
        """
        Extract Euler angles (Pitch, Yaw, Roll) from a rotation vector.

        Parameters
        ----------
        rvec : numpy.ndarray
            Rotation vector.

        Returns
        -------
        pitch : float
            Pitch angle in degrees.
        yaw : float
            Yaw angle in degrees.
        roll : float
            Roll angle in degrees.
        """
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        sy = np.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
            y = np.arctan2(-rotation_matrix[2, 0], sy)
            z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        else:
            x = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
            y = np.arctan2(-rotation_matrix[2, 0], sy)
            z = 0

        pitch = np.degrees(x)
        yaw = np.degrees(y)
        roll = np.degrees(z)
        return pitch, yaw, roll

    @staticmethod
    def extract_pitch_from_rotation(self, rvec):
        """
        Extract the pitch angle from a rotation vector.

        Parameters
        ----------
        rvec : numpy.ndarray
            Rotation vector.

        Returns
        -------
        pitch_degrees : float
            Pitch angle in degrees.
        """
        pitch, _, _ = self.extract_euler_angles(rvec)
        return pitch
    
    @staticmethod
    def calculate_rotation_angle(corner):
        # Calculate the center of the marker
        center_x = (corner[0][0] + corner[1][0] + corner[2][0] + corner[3][0]) / 4
        center_y = (corner[0][1] + corner[1][1] + corner[2][1] + corner[3][1]) / 4

        # Calculate the midpoint of one of the sides (between corner[0] and corner[1])
        midpoint_x = (corner[0][0] + corner[1][0]) / 2
        midpoint_y = (corner[0][1] + corner[1][1]) / 2

        # Calculate the vector from the center to the midpoint
        vector_x = midpoint_x - center_x
        vector_y = midpoint_y - center_y

        # Calculate the angle in radians and then convert to degrees
        angle_rad = np.arctan2(vector_y, vector_x)
        angle_deg = np.degrees(angle_rad)

        # Ensure the angle is in the range 0 to 360 degrees

        return angle_deg