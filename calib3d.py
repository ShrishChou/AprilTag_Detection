import numpy as np
import cv2
import glob
import os

# Checkerboard dimensions
CHECKERBOARD = (10, 7)  # adjust this to match your checkerboard
square_size = 0.025  # size of square in meters

# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real world space
imgpoints_left = []  # 2D points in image plane for left camera
imgpoints_right = []  # 2D points in image plane for right camera

# Get image files
images_left = glob.glob('calibration_images/camera2/*.jpg')
images_right = glob.glob('calibration_images/camera1/*.jpg')

assert len(images_left) == len(images_right), "Number of left and right images don't match"

for i, (img_left, img_right) in enumerate(zip(images_left, images_right)):
    left = cv2.imread(img_left)
    right = cv2.imread(img_right)
    gray_left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret_left, corners_left = cv2.findChessboardCorners(gray_left, CHECKERBOARD, None)
    ret_right, corners_right = cv2.findChessboardCorners(gray_right, CHECKERBOARD, None)

    if ret_left and ret_right:
        objpoints.append(objp)
        corners2_left = cv2.cornerSubPix(gray_left, corners_left, (11, 11), (-1, -1), criteria)
        corners2_right = cv2.cornerSubPix(gray_right, corners_right, (11, 11), (-1, -1), criteria)
        imgpoints_left.append(corners2_left)
        imgpoints_right.append(corners2_right)

        # Draw and display the corners
        cv2.drawChessboardCorners(left, CHECKERBOARD, corners2_left, ret_left)
        cv2.drawChessboardCorners(right, CHECKERBOARD, corners2_right, ret_right)
        cv2.imshow('Left Image', left)
        cv2.imshow('Right Image', right)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Calibrate left camera
ret_left, mtx_left, dist_left, rvecs_left, tvecs_left = cv2.calibrateCamera(objpoints, imgpoints_left, gray_left.shape[::-1], None, None)

# Calibrate right camera
ret_right, mtx_right, dist_right, rvecs_right, tvecs_right = cv2.calibrateCamera(objpoints, imgpoints_right, gray_right.shape[::-1], None, None)

# Validate the calibration by checking the reprojection error
mean_error_left = 0
mean_error_right = 0

for i in range(len(objpoints)):
    imgpoints2_left, _ = cv2.projectPoints(objpoints[i], rvecs_left[i], tvecs_left[i], mtx_left, dist_left)
    error_left = cv2.norm(imgpoints_left[i], imgpoints2_left, cv2.NORM_L2) / len(imgpoints2_left)
    mean_error_left += error_left

    imgpoints2_right, _ = cv2.projectPoints(objpoints[i], rvecs_right[i], tvecs_right[i], mtx_right, dist_right)
    error_right = cv2.norm(imgpoints_right[i], imgpoints2_right, cv2.NORM_L2) / len(imgpoints2_right)
    mean_error_right += error_right

mean_error_left /= len(objpoints)
mean_error_right /= len(objpoints)

print(f"Reprojection error for left camera: {mean_error_left}")
print(f"Reprojection error for right camera: {mean_error_right}")

# Stereo calibration
flags = 0
flags |= cv2.CALIB_FIX_INTRINSIC
criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# This step is performed to transformation between the two cameras and calculate Essential and Fundamental matrices
retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv2.stereoCalibrate(
    objpoints, imgpoints_left, imgpoints_right,
    mtx_left, dist_left,
    mtx_right, dist_right,
    gray_left.shape[::-1], criteria_stereo, flags)

print("Intrinsic Matrix Left:")
print(mtx_left)
print("\nDistortion Coefficients Left:")
print(dist_left)
print("\nIntrinsic Matrix Right:")
print(mtx_right)
print("\nDistortion Coefficients Right:")
print(dist_right)
print("\nRotation Matrix:")
print(rot)
print("\nTranslation Vector:")
print(trans)

# Save the calibration results
calibration_data = {
    'mtx1': mtx_left,
    'dist1': dist_left,
    'mtx2': mtx_right,
    'dist2': dist_right,
    'R': rot,
    'T': trans,
    'E': essentialMatrix,
    'F': fundamentalMatrix
}

np.savez('stereo_calibration.npz', **calibration_data)
print("\nCalibration data saved to 'stereo_calibration.npz'")
