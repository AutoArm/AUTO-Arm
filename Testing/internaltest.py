import cv2
import numpy as np

# Open the video capture
cap1 = cv2.VideoCapture(0, cv2.CAP_MSMF)

# Function to calculate the rotation angle
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

    return angle_deg, center_x, center_y, vector_x, vector_y

# ArUco dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

# Function to detect ArUco markers
def detect_aruco(image, target_id):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is not None and len(ids) > 0:
        ids = ids.flatten()
        if target_id in ids:
            index = np.where(ids == target_id)[0][0]
            return corners[index][0], int(ids[index])
    return None, None

target_id = 4

while True:
    ret1, frame1 = cap1.read()
    if not ret1:
        break

    corners, id = detect_aruco(frame1, target_id)
    if id is not None:
        rotation, center_x, center_y, vector_x, vector_y = calculate_rotation_angle(corners)
        print(f"Rotation Angle: {rotation:.2f} degrees")

        # Draw the detected markers
        cv2.aruco.drawDetectedMarkers(frame1, [np.array([corners])], np.array([id]))

        # Draw a line showing the orientation from the center
        end_x = int(center_x + vector_x * 2)
        end_y = int(center_y + vector_y * 2)
        cv2.line(frame1, (int(center_x), int(center_y)), (end_x, end_y), (0, 255, 0), 2)
        
        # Annotate the rotation angle on the image
        cv2.putText(frame1, f"Angle: {rotation:.2f} deg", (int(center_x), int(center_y) - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    else:
        print("ArUco tag not detected")

    cv2.imshow('Camera 1', frame1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cv2.destroyAllWindows()
