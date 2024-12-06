import cv2
import numpy as np

# Load calibration data (as before)
intrinsics_left = np.load('intrinsics_left.npz')
intrinsics_right = np.load('intrinsics_right.npz')
calibration_data = np.load('stereo_calibration2.npz')
mtx1 = calibration_data['mtx1']
dist1 = calibration_data['dist1']
mtx2 = calibration_data['mtx2']
dist2 = calibration_data['dist2']
R = calibration_data['R']
T = calibration_data['T']
actual_distance = 0.23  # 23 cm
calibrated_distance = 0.22  # 22 cm

# Calculate the scaling factor
scale_factor = actual_distance / calibrated_distance
print(f"Scaling factor: {scale_factor}")

# Adjust the translation vector
T = T * scale_factor
print(f"Scaled translation vector:\n{T}")

# Compute projection matrices
proj1 = mtx1 @ np.hstack((np.eye(3), np.zeros((3, 1))))
proj2 = mtx2 @ np.hstack((R, T))

# Define the ArUco dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

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

def estimate_pose(corners, mtx, dist, marker_length):
    # Define the 3D coordinates of the marker corners in the marker's coordinate system
    obj_points = np.array([
        [-marker_length / 2, marker_length / 2, 0],
        [marker_length / 2, marker_length / 2, 0],
        [marker_length / 2, -marker_length / 2, 0],
        [-marker_length / 2, -marker_length / 2, 0]
    ])

    # Solve for pose
    success, rvec, tvec = cv2.solvePnP(obj_points, corners, mtx, dist)
    return rvec, tvec

def draw_axis(img, rvec, tvec, mtx, dist):
    cv2.drawFrameAxes(img, mtx, dist, rvec, tvec, 0.1)  # length of the axis is 0.1 meter

def average_rotation_vectors(rvecs):
    rot_mats = [cv2.Rodrigues(rvec)[0] for rvec in rvecs]
    avg_rot_mat = np.mean(rot_mats, axis=0)
    U, _, Vt = np.linalg.svd(avg_rot_mat)
    avg_rot_mat = np.dot(U, Vt)
    avg_rvec, _ = cv2.Rodrigues(avg_rot_mat)
    return avg_rvec

# Main loop for real-time detection
cap1 = cv2.VideoCapture(1, cv2.CAP_MSMF)
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
cap2 = cv2.VideoCapture(2, cv2.CAP_MSMF)
cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
target_id = 4
marker_length = 0.062  # Length of the marker's side in meters

while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    if not ret1 or not ret2:
        print("Error: Could not read frame from one or both cameras.")
        break

    corners1, id1 = detect_aruco(frame1, target_id)
    corners2, id2 = detect_aruco(frame2, target_id)

    if corners1 is not None and corners2 is not None:
        rvec1, tvec1 = estimate_pose(corners1, mtx1, dist1, marker_length)
        rvec2, tvec2 = estimate_pose(corners2, mtx2, dist2, marker_length)

        print(f"Target ArUco tag detected!")
        print(f"Camera 1 - Rotation Vector:\n{rvec1}\nTranslation Vector:\n{tvec1}")
        print(f"Camera 2 - Rotation Vector:\n{rvec2}\nTranslation Vector:\n{tvec2}")

        draw_axis(frame1, rvec1, tvec1, mtx1, dist1)
        draw_axis(frame2, rvec2, tvec2, mtx2, dist2)

        avg_rvec = average_rotation_vectors([rvec1, rvec2])
        avg_tvec = np.mean([tvec1, tvec2], axis=0)

        print(f"Averaged Rotation Vector:\n{avg_rvec}\nAveraged Translation Vector:\n{avg_tvec}")

        draw_axis(frame1, avg_rvec, avg_tvec, mtx1, dist1)

        avg_rvec_text = f"Avg Rvec: {avg_rvec[0][0]:.2f}, {avg_rvec[1][0]:.2f}, {avg_rvec[2][0]:.2f}"
        avg_tvec_text = f"Avg Tvec: {avg_tvec[0][0]:.2f}, {avg_tvec[1][0]:.2f}, {avg_tvec[2][0]:.2f}"
        
        # Draw the text on the frame
        cv2.putText(frame1, avg_rvec_text, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame1, avg_tvec_text, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

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
