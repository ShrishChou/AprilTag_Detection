import numpy as np
import cv2
import glob

# Known intrinsic parameters
mtx_left = np.array([[2.58355258e+03, 0.00000000e+00, 3.35189421e+02],
 [0.00000000e+00, 2.57520269e+03, 1.97684233e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

dist_left = np.array([[-1.18098296e+01,  2.26853894e+02, -6.22676404e-02,  7.86331702e-02,
  -1.42272959e+03]])

mtx_right = mtx_left

dist_right = dist_left

# Checkerboard dimensions
CHECKERBOARD = (10, 7)  # adjust this to match your checkerboard
square_size = 25  # size of square in meters

# Prepare object points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real world space
imgpoints_left = []  # 2D points in image plane for left camera
imgpoints_right = []  # 2D points in image plane for right camera

# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Get image files
images_left = glob.glob('calibration_images/camera1/*.jpg')
images_right = glob.glob('calibration_images/camera2/*.jpg')

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
        corners2_left = cv2.cornerSubPix(gray_left, corners_left, (8, 8), (-1, -1), criteria)
        corners2_right = cv2.cornerSubPix(gray_right, corners_right, (8, 8), (-1, -1), criteria)
        imgpoints_left.append(corners2_left)
        imgpoints_right.append(corners2_right)

# Stereo calibration
flags = cv2.CALIB_FIX_INTRINSIC
criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# This step is performed to find transformation between the two cameras and calculate Essential and Fundamental matrices
retStereo, _, _, _, _, rot, trans, essentialMatrix, fundamentalMatrix = cv2.stereoCalibrate(
    objpoints, imgpoints_left, imgpoints_right, 
    mtx_left, dist_left, 
    mtx_right, dist_right, 
    gray_left.shape[::-1], 
    criteria_stereo, 
    flags)

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