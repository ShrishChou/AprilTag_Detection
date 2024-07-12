import cv2
import numpy as np
# Test Camera 1 with MSMF
cap1 = cv2.VideoCapture(1, cv2.CAP_MSMF) 
cap2 = cv2.VideoCapture(2, cv2.CAP_MSMF)  # For Windows MSMF
def detect_specific_qr_code(image, target_type, target_focus):
    qr_detector = cv2.QRCodeDetector()
    # Detect QR codes
    decoded, points, _ = qr_detector.detectAndDecode(image)
    
    if decoded and ':' in decoded:
        # Parse QR code data
        data = decoded.strip()
        object_type, focus_mm = data.split(':')
        focus_mm = float(focus_mm)
        
        # Check if this QR code matches our target
        if object_type == target_type and focus_mm == target_focus:
            # Calculate QR code center
            center_x = np.mean(points[:, 0])
            center_y = np.mean(points[:, 1])
            
            return (center_x, center_y), object_type, focus_mm
    
    return None, None, None
def detect_specific_apriltag(image, target_id):
    # Initialize the detector parameters using cv2.aruco module
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    parameters = cv2.aruco.DetectorParameters()
    
    # Create the ArucoDetector
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    
    # Detect AprilTags
    corners, ids, _ = detector.detectMarkers(image)
    
    if ids is not None and target_id in ids:
        # Find the index of the target_id in ids list
        index = np.where(ids == target_id)[0][0]
        
        # Calculate center of the AprilTag
        center_x = int(np.mean(corners[index][:, 0]))
        center_y = int(np.mean(corners[index][:, 1]))
        
        return (center_x, center_y), int(ids[index][0])
    
    return None, None
if not cap1.isOpened():
    print("Error: Could not open camera 1.")
    cap1.release()
    cv2.destroyAllWindows()
    exit()
if not cap2.isOpened():
    print("Error: Could not open camera 2.")
    cap2.release()
    cv2.destroyAllWindows()
    exit()

while True:
    ret1, frame1 = cap1.read()
    ret2,frame2= cap2.read()
    if not ret1:
        print("Error: Could not read frame from camera 1.")
        break
    if not ret2:
        print("Error: Could not read frame from camera 2.")
        break
    cv2.imshow('Camera 1', frame1)
    cv2.imshow('Camera 2', frame2)
    one=detect_specific_apriltag(frame1,0)
    two=detect_specific_apriltag(frame2,0)
    if one  != (None,None):
        print(one)
    if two  != (None,None):
        print(two)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cv2.destroyAllWindows()
