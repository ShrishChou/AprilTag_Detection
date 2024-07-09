import cv2
import numpy as np
import glob
import os

# Define the chessboard size (number of inner corners per a chessboard row and column)
chessboard_size = (10, 7)

# Define the size of a square on your chessboard in millimeters
square_size = 25  # Example: 25 mm

# Termination criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (3D points in the real world)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Path to the images directory
image_directory = os.path.join('images', '*.jpg')

# Load calibration images from the directory
images = glob.glob(image_directory)

# Check if images were found
if not images:
    print("No images found in the specified directory.")
else:
    print(f"Found {len(images)} images.")

for image_file in images:
    img = cv2.imread(image_file)
    
    # Check if the image was loaded properly
    if img is None:
        print(f"Error loading image: {image_file}")
        continue
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    
    if ret:
        objpoints.append(objp)
        
        # Refine the corner positions
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        
        # Draw and display the corners
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow('Chessboard', img)
        cv2.waitKey(500)  # Display each image for 500 ms
    else:
        print(f"Chessboard corners not found in image: {image_file}")
        # Display the image to see why corners were not detected
        cv2.imshow('Chessboard', img)
        cv2.waitKey(500)
        
cv2.destroyAllWindows()

# Check if there are enough valid image points for calibration
if len(objpoints) > 0 and len(imgpoints) > 0:
    # Calibrate the camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Intrinsic parameters
    focal_length_x = mtx[0, 0]
    focal_length_y = mtx[1, 1]
    focal_center_x = mtx[0, 2]
    focal_center_y = mtx[1, 2]

    print(dist)
    print(mtx)
    print("Focal Length (X):", focal_length_x)
    print("Focal Length (Y):", focal_length_y)
    print("Focal Center (X):", focal_center_x)
    print("Focal Center (Y):", focal_center_y)
    
    # Calculate re-projection error
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error
    mean_error = total_error / len(objpoints)
    print("Mean re-projection error:", mean_error)
    # Load an image to undistort
    test_image_path = 'images/WIN_20240707_17_29_23_Pro.jpg'  # Path to your test image
    img = cv2.imread(test_image_path)

    # Undistort the image
    h, w = img.shape[:2]
    new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    undistorted_img = cv2.undistort(img, mtx, dist, None, new_camera_mtx)

    # Crop the image based on the ROI (Region of Interest)
    x, y, w, h = roi
    undistorted_img = undistorted_img[y:y+h, x:x+w]

    # Display the original and undistorted images
    cv2.imshow('Original Image', img)
    cv2.imshow('Undistorted Image', undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Not enough valid image points for calibration.")

