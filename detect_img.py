import cv2
import numpy as np

# Stereo calibration parameters
# These should be loaded from your calibration process
calibration_data = np.load('stereo_calibration2.npz')
cam_matrix_1 = calibration_data['mtx1']
dist_coeffs_1 = calibration_data['dist1']
cam_matrix_2 = calibration_data['mtx2']
dist_coeffs_2 = calibration_data['dist2']
R = calibration_data['R']
T = calibration_data['T']

# Open video streams
# For webcams, use 0, 1, etc. For video files, use the file paths
cap_left = cv2.VideoCapture(1) 
cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
cap_right = cv2.VideoCapture(2)  # For Windows MSMF
cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
# Detect ArUco markers
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

def detect_aruco(image, target_id):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Detect ArUco markers
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None and len(ids) > 0:
        ids = ids.flatten()
        
        if target_id in ids:
            index = np.where(ids == target_id)[0][0]
            
            # Get the corners of the ArUco marker
            marker_corners = corners[index]
            
            return marker_corners, int(ids[index])
    
    return None, None

# Compute rectification parameters once
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
    cam_matrix_1, dist_coeffs_1, cam_matrix_2, dist_coeffs_2, 
    (3840, 2160), R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=-1)

while True:
    # Capture frames
    ret_left, frame_left = cap_left.read()
    ret_right, frame_right = cap_right.read()
    
    if not ret_left or not ret_right:
        print("Failed to grab frames")
        break
    
    corners_left, id_left = detect_aruco(frame_left, target_id=4)  # Assuming target_id is 0
    corners_right, id_right = detect_aruco(frame_right, target_id=4)
    # Ensure the same markers are detected in both frames
    if corners_left is not None and corners_right is not None:
        # Both cameras detected the marker
        pts_left = corners_left[0][0].T
        pts_right = corners_right[0][0].T
        
        # Undistort points
        pts_left = cv2.undistortPoints(np.expand_dims(pts_left, axis=1), cam_matrix_1, dist_coeffs_1, P=P1)
        pts_right = cv2.undistortPoints(np.expand_dims(pts_right, axis=1), cam_matrix_2, dist_coeffs_2, P=P2)

        # Triangulate points
        points_4D = cv2.triangulatePoints(P1, P2, pts_left, pts_right)

        # Convert to 3D
        points_3D = points_4D[:3] / points_4D[3]

        # Calculate the center of the ArUco marker
        center_3D = np.mean(points_3D, axis=1)

        print(f"3D position of the ArUco tag: {center_3D}")
    else:
        print("Marker not detected in both cameras")
    # Display the frames
    cv2.namedWindow('Camera 1', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Camera 1', 3840, 2160)  # Adjust to your screen size
    cv2.imshow('Camera 1', frame_left)
    cv2.namedWindow('Camera 2', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Camera 2', 3840, 2160)  # Adjust to your screen size
    cv2.imshow('Camera 2', frame_right)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture objects
cap_left.release()
cap_right.release()
cv2.destroyAllWindows()