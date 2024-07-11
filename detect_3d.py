import cv2
import numpy as np
from pyzbar import pyzbar

# Load calibration data (as before)
calibration_data = np.load('stereo_calibration.npz')
mtx1 = calibration_data['mtx1']
dist1 = calibration_data['dist1']
mtx2 = calibration_data['mtx2']
dist2 = calibration_data['dist2']
R = calibration_data['R']
T = calibration_data['T']

# Compute projection matrices
proj1 = np.hstack((np.eye(3, 3), np.zeros((3, 1))))
proj2 = np.hstack((R, T))

def detect_specific_qr_code(image, target_type, target_focus):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    qr_codes = pyzbar.decode(gray)
    
    for qr_code in qr_codes:
        # Get QR code data and position
        data = qr_code.data.decode('utf-8')
        object_type, focus_mm = data.split(':')
        focus_mm = float(focus_mm)
        
        # Check if this QR code matches our target
        if object_type == target_type and focus_mm == target_focus:
            # Get QR code center
            (x, y, w, h) = qr_code.rect
            center_x = x + w / 2
            center_y = y + h / 2
            
            return (center_x, center_y), object_type, focus_mm
    
    return None, None, None

def triangulate_point(point1, point2):
    point1 = np.array(point1).reshape(2, 1)
    point2 = np.array(point2).reshape(2, 1)
    
    # Undistort points
    point1_undistorted = cv2.undistortPoints(point1, mtx1, dist1)
    point2_undistorted = cv2.undistortPoints(point2, mtx2, dist2)
    
    # Triangulate
    point_4d = cv2.triangulatePoints(proj1, proj2, point1_undistorted, point2_undistorted)
    
    # Convert from homogeneous coordinates to 3D
    point_3d = point_4d[:3] / point_4d[3]
    
    return point_3d.reshape(-1)

# Main loop for real-time detection
cap1 = cv2.VideoCapture(1)  # Camera 1
cap2 = cv2.VideoCapture(2)  # Camera 2

# Specify the target QR code
target_type = "VALVE"
target_focus = 50.0

print(f"Looking for QR code with Type: {target_type}, Focus: {target_focus}mm")

while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()
    
    if not ret1 or not ret2:
        break
    
    # Detect specific QR code in both frames
    point1, type1, focus1 = detect_specific_qr_code(frame1, target_type, target_focus)
    point2, type2, focus2 = detect_specific_qr_code(frame2, target_type, target_focus)
    
    if point1 is not None and point2 is not None:
        # Triangulate 3D position
        point_3d = triangulate_point(point1, point2)
        print(f"Target QR Code detected!")
        print(f"3D position: X={point_3d[0]:.2f}, Y={point_3d[1]:.2f}, Z={point_3d[2]:.2f}")
    
    # Display frames (optional)
    cv2.imshow('Camera 1', frame1)
    cv2.imshow('Camera 2', frame2)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cap2.release()
cv2.destroyAllWindows()