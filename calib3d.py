import numpy as np
import cv2
import glob

# Chessboard dimensions
CHESSBOARD_SIZE = (10, 7)  # (width, height) of intersections
SQUARE_SIZE = 25.0  # Size of a square in millimeters

# Prepare object points
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1,2) * SQUARE_SIZE

# Arrays to store object points and image points from all images
objpoints = []  # 3d points in real world space
imgpoints1 = []  # 2d points in image plane for camera 1
imgpoints2 = []  # 2d points in image plane for camera 2

# Get lists of calibration images
images1 = glob.glob('calibration_images/camera1/*.jpg')
images2 = glob.glob('calibration_images/camera2/*.jpg')

# Ensure we have the same number of images for both cameras
assert len(images1) == len(images2), "Number of images from both cameras must be the same"

for img1_path, img2_path in zip(images1, images2):
    img1 = cv2.imread(img1_path)
    img2 = cv2.imread(img2_path)
    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret1, corners1 = cv2.findChessboardCorners(gray1, CHESSBOARD_SIZE, None)
    ret2, corners2 = cv2.findChessboardCorners(gray2, CHESSBOARD_SIZE, None)

    # If found, add object points, image points
    if ret1 and ret2:
        objpoints.append(objp)
        imgpoints1.append(corners1)
        imgpoints2.append(corners2)

        # Draw and display the corners (optional)
        cv2.drawChessboardCorners(img1, CHESSBOARD_SIZE, corners1, ret1)
        cv2.drawChessboardCorners(img2, CHESSBOARD_SIZE, corners2, ret2)
        cv2.imshow('img1', img1)
        cv2.imshow('img2', img2)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Calibrate each camera individually
ret1, mtx1, dist1, rvecs1, tvecs1 = cv2.calibrateCamera(objpoints, imgpoints1, gray1.shape[::-1], None, None)
ret2, mtx2, dist2, rvecs2, tvecs2 = cv2.calibrateCamera(objpoints, imgpoints2, gray2.shape[::-1], None, None)

print("Camera 1 Matrix:")
print(mtx1)
print("\nCamera 1 Distortion Coefficients:")
print(dist1)

print("\nCamera 2 Matrix:")
print(mtx2)
print("\nCamera 2 Distortion Coefficients:")
print(dist2)

# Perform stereo calibration
flags = cv2.CALIB_FIX_INTRINSIC
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

retStereo, newCameraMatrix1, dist1, newCameraMatrix2, dist2, R, T, E, F = cv2.stereoCalibrate(
    objpoints, imgpoints1, imgpoints2, 
    mtx1, dist1, mtx2, dist2, 
    gray1.shape[::-1], criteria=criteria, flags=flags)

print("\nRotation Matrix:")
print(R)
print("\nTranslation Vector:")
print(T)

# Calculate and print baseline
baseline = np.linalg.norm(T)
print(f"\nThe baseline distance between cameras is {baseline:.2f} millimeters")
expected_baseline = 812.8  # 32 inches in mm
scale_factor = expected_baseline / baseline
print(f"Scale factor: {scale_factor:.4f}")


print("\nOriginal Translation Vector:")
print(T)


# Save the calibration results
np.savez('stereo_calibration.npz', 
         mtx1=newCameraMatrix1, dist1=dist1, 
         mtx2=newCameraMatrix2, dist2=dist2, 
         R=R, T=T, E=E, F=F, baseline=baseline)

print("\nCalibration complete. Results saved to 'stereo_calibration.npz'")

# Optional: Compute reprojection error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs1[i], tvecs1[i], mtx1, dist1)
    error = cv2.norm(imgpoints1[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error

print(f"\nTotal average reprojection error: {mean_error/len(objpoints):.5f}")