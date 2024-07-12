import robotpy_apriltag
import cv2

# Code to detect tag and make outlines (no pose work)

cap = cv2.VideoCapture(2)
detector=robotpy_apriltag.AprilTagDetector()
works=detector.addFamily("tag36h11")

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    gray=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags=detector.detect(gray)
    # loop through each tag
    for tag in tags:
        # Mark corners
        corners=tag.getCorners((1,2,3,4,5,6,7,8))
        for i in range(3):
            pt1=(int(corners[2*i]),int(corners[2*i+1]))
            pt2=(int(corners[2*(i+1)]),int(corners[2*(i+1)+1]))
            cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
        pt1 = (int(corners[0]),int(corners[1]))
        pt4 = (int(corners[6]),int(corners[7]))
        cv2.line(frame, pt1, pt4, (0, 255, 0), 2)

        # Mark center
        center=tag.getCenter()
        plotcenter=(int(center.x),int(center.y))
        cv2.circle(frame, plotcenter, 5, (0, 0, 255), -1)

        # Display the tag ID
        cv2.putText(frame, str(tag.getFamily()), (plotcenter[0] - 10, plotcenter[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # Display the resulting frame
    cv2.imshow('AprilTag Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture and close windows
cap.release()
cv2.destroyAllWindows()