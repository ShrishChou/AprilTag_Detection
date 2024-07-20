import numpy as np
import cv2
import glob

# Load intrinsic parameters
intrinsics_left = np.load('intrinsics_left.npz')
intrinsics_right = np.load('intrinsics_right.npz')
mtx_left = intrinsics_left['mtx_left']
dist_left = intrinsics_left['dist_left']
mtx_right = intrinsics_right['mtx_right']
dist_right = intrinsics_right['dist_right']

# Checkerboard dimensions
CHECKERBOARD = (10, 7)
square_size = 0.025  # Size of a square in meters

# Termination criteria for corner sub-pixel accuracy
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points
objpoints = []  # 3D points in real-world space
imgpoints_left = []  # 2D points in image plane for left camera
imgpoints_right = []  # 2D points in image plane for right camera

# Get image files for left and right cameras
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
        corners2_left = cv2.cornerSubPix(gray_left, corners_left, (11, 11), (-1, -1), criteria)
        corners2_right = cv2.cornerSubPix(gray_right, corners_right, (11, 11), (-1, -1), criteria)
        imgpoints_left.append(corners2_left)
        imgpoints_right.append(corners2_right)

cv2.destroyAllWindows()

# Stereo calibration flags
flags = 0
flags |= cv2.CALIB_FIX_INTRINSIC
criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Perform stereo calibration
retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv2.stereoCalibrate(
    objpoints, imgpoints_left, imgpoints_right,
    mtx_left, dist_left,
    mtx_right, dist_right,
    gray_left.shape[::-1], criteria_stereo, flags)

# Save stereo calibration results
np.savez('stereo_calibration.npz', R=rot, T=trans, E=essentialMatrix, F=fundamentalMatrix)

print("Stereo calibration results saved as 'stereo_calibration.npz'")
print("Rotation Matrix:\n", rot)
print("Translation Vector:\n", trans)
