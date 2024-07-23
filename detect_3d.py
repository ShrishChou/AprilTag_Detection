import cv2
import numpy as np

# Load calibration data
intrinsics_left = np.load('intrinsics_left.npz')
intrinsics_right = np.load('intrinsics_right.npz')
calibration_data = np.load('stereo_calibration.npz')
mtx1 = calibration_data['mtx1']
dist1 = calibration_data['dist1']
mtx2 = calibration_data['mtx2']
dist2 = calibration_data['dist2']
R = calibration_data['R']
T = calibration_data['T']

# Compute projection matrices
proj1 = mtx1 @ np.hstack((np.eye(3), np.zeros((3, 1))))
proj2 = mtx2 @ np.hstack((R, T))

# Define the ArUco dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

# Define the ArUco marker size (in meters)
aruco_size = 0.065

# Define the 3D points of the ArUco marker corners (assuming square markers)
aruco_corners_3d = np.array([
    [-aruco_size / 2, -aruco_size / 2, 0],
    [aruco_size / 2, -aruco_size / 2, 0],
    [aruco_size / 2, aruco_size / 2, 0],
    [-aruco_size / 2, aruco_size / 2, 0]
], dtype=np.float32)

# Scaling factor
scale_factor = 1.22 / 0.74

# Main loop for real-time detection
cap1 = cv2.VideoCapture(1, cv2.CAP_MSMF)
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
cap2 = cv2.VideoCapture(2, cv2.CAP_MSMF)
cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)

target_id = 4
def average_pose(rvec1, tvec1, rvec2, tvec2):
    # Average the translation vectors
    tvec_avg = (tvec1 + tvec2) / 2

    # Convert rotation vectors to matrices
    R1, _ = cv2.Rodrigues(rvec1)
    R2, _ = cv2.Rodrigues(rvec2)

    # Average the rotation matrices
    R_avg = (R1 + R2) / 2

    # Ensure R_avg is a valid rotation matrix by orthogonalizing it using SVD
    U, _, Vt = np.linalg.svd(R_avg)
    R_avg_orthogonal = U @ Vt

    # Convert the averaged rotation matrix back to a rotation vector
    rvec_avg, _ = cv2.Rodrigues(R_avg_orthogonal)

    return rvec_avg, tvec_avg

while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    if not ret1 or not ret2:
        print("Error: Could not read frame from one or both cameras.")
        break

    gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners1, ids1, _ = detector.detectMarkers(gray1)
    corners2, ids2, _ = detector.detectMarkers(gray2)
    pose1=None
    pose2=None
    if ids1 is not None and target_id in ids1.flatten():
        idx1 = np.where(ids1.flatten() == target_id)[0][0]
        corners1_target = corners1[idx1].reshape(-1, 2)

        # Estimate pose using solvePnP
        success1, rvec1, tvec1 = cv2.solvePnP(aruco_corners_3d, corners1_target, mtx1, dist1)

        if success1:
            # Scale and adjust the position
            tvec1 = tvec1 * scale_factor
            

            # Draw the detected marker and axes
            cv2.aruco.drawDetectedMarkers(frame1, corners1)
            cv2.drawFrameAxes(frame1, mtx1, dist1, rvec1, tvec1, 0.1)
            tvec1[2] = 1.3-tvec1[2]
            # Convert rotation vector to a rotation matrix
            R1, _ = cv2.Rodrigues(rvec1)
            pose_matrix1 = np.hstack((R1, tvec1))

            # Print position and orientation
            print(f"Position (Camera 1): {tvec1.flatten()}")
            pose1=tvec1.flatten()
            print(f"Rotation Matrix (Camera 1):\n{R1}")

    if ids2 is not None and target_id in ids2.flatten():
        idx2 = np.where(ids2.flatten() == target_id)[0][0]
        corners2_target = corners2[idx2].reshape(-1, 2)

        # Estimate pose using solvePnP
        success2, rvec2, tvec2 = cv2.solvePnP(aruco_corners_3d, corners2_target, mtx2, dist2)

        if success2:
            # Scale and adjust the position
            tvec2 = tvec2 * scale_factor

            # Draw the detected marker and axes
            cv2.aruco.drawDetectedMarkers(frame2, corners2)
            cv2.drawFrameAxes(frame2, mtx2, dist2, rvec2, tvec2, 0.1)
            tvec2[2] = 1.3-tvec2[2]

            # Convert rotation vector to a rotation matrix
            R2, _ = cv2.Rodrigues(rvec2)
            pose_matrix2 = np.hstack((R2, tvec2))

            # Print position and orientation
            print(f"Position (Camera 2): {tvec2.flatten()}")
            pose2=tvec2.flatten()
            print(f"Rotation Matrix (Camera 2):\n{R2}")
    if pose1 is not None and pose2 is not None:

        rvec_avg, tvec_avg = average_pose(rvec1, tvec1, rvec2, tvec2)
        print(f"Averaged Position:{tvec_avg.flatten()}")
        print(f"Averaged Rotation Vector: {rvec_avg.flatten()}")
    # Display frames
    cv2.namedWindow('Camera 1', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Camera 1', 3840, 2160)
    cv2.imshow('Camera 1', frame1)
    cv2.namedWindow('Camera 2', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Camera 2', 3840, 2160)
    cv2.imshow('Camera 2', frame2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cap2.release()
cv2.destroyAllWindows()
