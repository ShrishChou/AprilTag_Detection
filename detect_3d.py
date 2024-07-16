import cv2
import numpy as np

# Load calibration data (as before)
calibration_data = np.load('stereo_calibration.npz')
mtx1 = calibration_data['mtx1']
dist1 = calibration_data['dist1']
mtx2 = calibration_data['mtx2']
dist2 = calibration_data['dist2']
R = calibration_data['R']
T = calibration_data['T']
# At the beginning of your script, after loading the calibration data:
print("Calibration data:")
print(f"Camera 1 matrix:\n{mtx1}")
print(f"Camera 1 distortion:\n{dist1}")
print(f"Camera 2 matrix:\n{mtx2}")
print(f"Camera 2 distortion:\n{dist2}")
print(f"Rotation matrix:\n{R}")
print(f"Translation vector:\n{T}")
# Compute projection matrices
proj1 = np.hstack((np.eye(3), np.zeros((3, 1))))
proj2 = np.hstack((R, T))

# Define the ArUco dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

def detect_aruco(image, target_id):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Create the ArucoDetector
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    
    # Detect ArUco markers
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None and len(ids) > 0:
        ids = ids.flatten()
        
        if target_id in ids:
            index = np.where(ids == target_id)[0][0]
            
            # Calculate center of the ArUco marker
            center_x = int(np.mean(corners[index][0][:, 0]))
            center_y = int(np.mean(corners[index][0][:, 1]))
            
            return (center_x, center_y), int(ids[index])
    
    return None, None

def triangulate_point(point1, point2):
    point1 = np.array(point1, dtype=np.float32).reshape(1, 1, 2)
    point2 = np.array(point2, dtype=np.float32).reshape(1, 1, 2)

    # Undistort points
    point1_undistorted = cv2.undistortPoints(point1, mtx1, dist1, P=mtx1)
    point2_undistorted = cv2.undistortPoints(point2, mtx2, dist2, P=mtx2)

    print(f"Original points - Camera 1: {point1}, Camera 2: {point2}")
    print(f"Undistorted points - Camera 1: {point1_undistorted}, Camera 2: {point2_undistorted}")

    # Triangulate
    point_4d = cv2.triangulatePoints(proj1, proj2, point1_undistorted, point2_undistorted)

    # Convert from homogeneous coordinates to 3D
    point_3d = point_4d[:3] / point_4d[3]

    return point_3d.reshape(-1)

def calculate_reprojection_error(point_3d, point1, point2, mtx1, dist1, mtx2, dist2):
    point_3d_homogeneous = np.append(point_3d, 1)

    reprojected_point1, _ = cv2.projectPoints(point_3d_homogeneous[:3], np.eye(3), np.zeros(3), mtx1, dist1)
    reprojected_point2, _ = cv2.projectPoints(point_3d_homogeneous[:3], R, T, mtx2, dist2)

    print(f"Original points: Camera 1 {point1}, Camera 2 {point2}")
    print(f"Reprojected points: Camera 1 {reprojected_point1.squeeze()}, Camera 2 {reprojected_point2.squeeze()}")

    error1 = np.linalg.norm(np.array(point1) - reprojected_point1.squeeze())
    error2 = np.linalg.norm(np.array(point2) - reprojected_point2.squeeze())

    return error1, error2
# Main loop for real-time detection
cap1 = cv2.VideoCapture(0)  # Camera 1
cap2 = cv2.VideoCapture(2)  # Camera 2

# Specify the target ArUco tag ID
target_id = 4  # Change this to your desired tag ID

print(f"Looking for ArUco tag with ID: {target_id}")

while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    if not ret1 or not ret2:
        print("Error: Could not read frame from one or both cameras.")
        break

    # Detect specific ArUco tag in both frames
    point1, id1 = detect_aruco(frame1, target_id)
    point2, id2 = detect_aruco(frame2, target_id)

    if point1 is not None and point2 is not None:
        point_3d = triangulate_point(point1, point2)
        print(f"Target ArUco tag detected!")
        print(f"3D position: X={point_3d[0]:.2f}, Y={point_3d[1]:.2f}, Z={point_3d[2]:.2f}")

        error1, error2 = calculate_reprojection_error(point_3d, point1, point2, mtx1, dist1, mtx2, dist2)
        print(f"Reprojection error (Camera 1): {error1:.2f}")
        print(f"Reprojection error (Camera 2): {error2:.2f}")

        # Draw the detected tag on the frames
        cv2.circle(frame1, point1, 5, (0, 255, 0), -1)
        cv2.circle(frame2, point2, 5, (0, 255, 0), -1)

    # Display frames
    cv2.imshow('Camera 1', frame1)
    cv2.imshow('Camera 2', frame2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cap2.release()
cv2.destroyAllWindows()