import cv2
import numpy as np

# [Keep the existing code for loading calibration data and setting up cameras]

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

    return np.array(points_3d)

# Main loop for real-time detection
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
            print(f"Corner {i+1}: X={point[0]:.2f}, Y={point[1]:.2f}, Z={point[2]:.2f}")

        # Draw the detected tag corners on the frames
        cv2.drawContours(frame1, [corners1.astype(int)], 0, (0, 255, 0), 2)
        cv2.drawContours(frame2, [corners2.astype(int)], 0, (0, 255, 0), 2)
        
        # Draw corner numbers
        for i, corner in enumerate(corners1):
            cv2.putText(frame1, str(i+1), tuple(corner.astype(int)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        for i, corner in enumerate(corners2):
            cv2.putText(frame2, str(i+1), tuple(corner.astype(int)), 
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