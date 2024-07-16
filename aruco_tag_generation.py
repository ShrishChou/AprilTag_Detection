import cv2
import numpy as np

# Define the dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# Create an image from the marker
tag_size = 400  # Size of the tag in pixels
tag_image = np.zeros((tag_size, tag_size, 1), dtype=np.uint8)
cv2.aruco.generateImageMarker(aruco_dict, 4, tag_size, tag_image, 1)

# Save the image
cv2.imwrite("aruco_tag_4.png", tag_image)

# Optional: Display the image
cv2.imshow("ArUco Tag 4", tag_image)
cv2.waitKey(0)
cv2.destroyAllWindows()