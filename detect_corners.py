import cv2
import numpy as np

# [Keep the existing code for loading calibration data and setting up cameras]
# Load calibration data (as before)
intrinsics_left = np.load('intrinsics_left.npz')
intrinsics_right = np.load('intrinsics_right.npz')
calibration_data=np.load('stereo_calibration.npz')
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
proj1 = mtx1 @ np.hstack((np.eye(3), np.zeros((3, 1))))
proj2 = mtx2 @ np.hstack((R, T))

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
            
            # Return the corners of the ArUco marker
            return corners[index][0], int(ids[index])
    
    return None, None

def triangulate_points(corners1, corners2):
    points_3d = []
    for pt1, pt2 in zip(corners1, corners2):
        pt1 = np.array(pt1, dtype=np.float32).reshape(1, 1, 2)
        pt2 = np.array(pt2, dtype=np.float32).reshape(1, 1, 2)

        # Undistort points
        pt1_undistorted = cv2.undistortPoints(pt1, mtx1, dist1, P=mtx1)
        pt2_undistorted = cv2.undistortPoints(pt2, mtx2, dist2, P=mtx2)

        # Triangulate
        point_4d = cv2.triangulatePoints(proj1, proj2, pt1_undistorted, pt2_undistorted)

        # Convert from homogeneous coordinates to 3D
        point_3d = (point_4d[:3] / point_4d[3]).reshape(-1)

        points_3d.append(point_3d)

    points_3d = np.array(points_3d)
    
    # Calculate scale based on the known size of the ArUco marker
    aruco_size = 0.065  # Adjust this to the actual size of your ArUco marker in meters
    edge_lengths = np.linalg.norm(points_3d[1:] - points_3d[:-1], axis=1)
    scale_factor = aruco_size / np.mean(edge_lengths)
    
    points_3d_scaled = points_3d * scale_factor
    
    return points_3d_scaled


# Main loop for real-time detection
cap1 = cv2.VideoCapture(1,cv2.CAP_MSMF) 
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
cap2 = cv2.VideoCapture(2,cv2.CAP_MSMF)  # For Windows MSMF
cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
target_id=4
while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    if not ret1 or not ret2:
        print("Error: Could not read frame from one or both cameras.")
        break

    # Detect specific ArUco tag in both frames
    corners1, id1 = detect_aruco(frame1, target_id)
    corners2, id2 = detect_aruco(frame2, target_id)

    if corners1 is not None and corners2 is not None:
        points_3d = triangulate_points(corners1, corners2)
        
        print(f"Target ArUco tag detected!")
        print(f"3D positions of corners:")
        for i, point in enumerate(points_3d):
            print(f"Corner {i+1}: X={point[0]:.3f}, Y={point[1]:.3f}, Z={point[2]:.3f}")

        # Calculate marker orientation
        edge1 = points_3d[1] - points_3d[0]
        edge2 = points_3d[3] - points_3d[0]
        normal = np.cross(edge1, edge2)
        normal = normal / np.linalg.norm(normal)

        # Calculate and print the estimated distance to the marker
        center_3d = np.mean(points_3d, axis=0)
        distance = np.linalg.norm(center_3d)
        print(f"Estimated distance to marker: {distance:.3f} meters")
        print(f"Marker normal vector: {normal}")
        
        # Calculate angle between marker normal and camera axis
        camera_axis = np.array([0, 0, 1])
        angle = np.arccos(np.dot(normal, camera_axis)) * 180 / np.pi
        print(f"Angle between marker and camera: {angle:.2f} degrees")

        # After calculating the normal vector and center_3d
        normal_length = 0.2  # Length of the normal vector to draw
        normal_end = center_3d + normal * normal_length

        # Project these 3D points back to 2D
        center_2d, _ = cv2.projectPoints(center_3d.reshape(1,3), np.zeros(3), np.zeros(3), mtx1, dist1)
        normal_end_2d, _ = cv2.projectPoints(normal_end.reshape(1,3), np.zeros(3), np.zeros(3), mtx1, dist1)

        # Draw the normal vector on the image
        cv2.line(frame1, tuple(center_2d[0][0].astype(int)), tuple(normal_end_2d[0][0].astype(int)), (0, 0, 255), 2)

        # Add text for distance and angle
        cv2.putText(frame1, f"Distance: {distance:.2f}m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame1, f"Angle: {angle:.2f} deg", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        # Draw the detected tag corners on the frames
        cv2.drawContours(frame1, [corners1.astype(int)], 0, (0, 255, 0), 2)
        cv2.drawContours(frame2, [corners2.astype(int)], 0, (0, 255, 0), 2)
        
        # Draw corner numbers
        for i, corner in enumerate(corners1):
            cv2.putText(frame1, f"{i+1}: {corner[0]:.0f},{corner[1]:.0f}", tuple(corner.astype(int)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        for i, corner in enumerate(corners2):
            cv2.putText(frame2, f"{i+1}: {corner[0]:.0f},{corner[1]:.0f}", tuple(corner.astype(int)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    # Display frames
    cv2.namedWindow('Camera 1', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Camera 1', 3840, 2160)  # Adjust to your screen size
    cv2.imshow('Camera 1', frame1)
    cv2.namedWindow('Camera 2', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Camera 2', 3840, 2160)  # Adjust to your screen size
    cv2.imshow('Camera 2', frame2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cap2.release()
cv2.destroyAllWindows()