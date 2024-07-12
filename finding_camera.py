import cv2

# Test Camera 1 with MSMF
cap1 = cv2.VideoCapture(1, cv2.CAP_MSMF) 
cap2 = cv2.VideoCapture(2, cv2.CAP_MSMF)  # For Windows MSMF
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

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cv2.destroyAllWindows()
