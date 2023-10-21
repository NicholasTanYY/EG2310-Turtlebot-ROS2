"""This python file converts a 2D map into a binary map to be used as the occupancy map"""

import cv2

# Load the pre-built map as an image
map_image = cv2.imread('prebuilt_map.png')

cv2.imshow("pre built map", map_image)
cv2.waitKey(0)
# Convert the image to grayscale
gray_image = cv2.cvtColor(map_image, cv2.COLOR_BGR2GRAY)

# Threshold the grayscale image to create a binary map
ret, binary_map = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)

# Save the binary map as an image
cv2.imwrite('binary_map.png', binary_map)
