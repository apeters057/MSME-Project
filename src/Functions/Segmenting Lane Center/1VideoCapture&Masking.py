import sys
import cv2
import numpy as np

# Adjust paths as needed
sys.path.append('/home/apeters/project/env/src/Functions')
sys.path.append('/home/apeters/project/env/src')
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')
sys.path.append('/home/apeters/project/env/src/robot-hat')

# Capture video from the camera
cap = cv2.VideoCapture(0)


def nothing(x):
    pass

# Create a window
cv2.namedWindow("Trackbars")

#### Create trackbars for adjusting HSV values ####
# "Hue" Represents the color type
# "Saturation" Represents the intensity or purity of the color
#    Low saturation results in more greyish colors.
# "Value" Represents brightness (0-255 scale).
#    Higher values mean brighter colors.
cv2.createTrackbar("Lower Hue", "Trackbars", 29, 180, nothing)
cv2.createTrackbar("Lower Saturation", "Trackbars", 70, 255, nothing)
cv2.createTrackbar("Lower Value", "Trackbars", 135, 255, nothing)
cv2.createTrackbar("Upper Hue", "Trackbars", 69, 180, nothing)
cv2.createTrackbar("Upper Saturation", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("Upper Value", "Trackbars", 255, 255, nothing)
#cv2.createTrackbar("Lower Canny", "Trackbars", 50, 200, nothing)
#cv2.createTrackbar("Upper Canny", "Trackbars", 150, 200, nothing)

#Better Value
#LH = 29
#LS =70
#LV = 135
#UH = 069
#US = 255
#UV = 255

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
#Find Pixel Lane Color
    # Get current positions of the trackbars
    lh = cv2.getTrackbarPos("Lower Hue", "Trackbars")
    ls = cv2.getTrackbarPos("Lower Saturation", "Trackbars")
    lv = cv2.getTrackbarPos("Lower Value", "Trackbars")
    uh = cv2.getTrackbarPos("Upper Hue", "Trackbars")
    us = cv2.getTrackbarPos("Upper Saturation", "Trackbars")
    uv = cv2.getTrackbarPos("Upper Value", "Trackbars")
    lc = cv2.getTrackbarPos("Lower Canny", "Trackbars")
    uc = cv2.getTrackbarPos("Upper Canny", "Trackbars")

    # Define lower and upper range of green in HSV
    lower_green = np.array([lh, ls, lv])
    upper_green = np.array([uh, us, uv])

    # Convert frame to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Apply mask to isolate green color
    mask = cv2.inRange(hsv_frame, lower_green, upper_green)

    # Optional: Apply Gaussian Blur to reduce noise in the mask
    mask = cv2.GaussianBlur(mask, (5, 5), 0)

    # Display the original frame and mask for debugging
    cv2.imshow("Original", frame)
    cv2.imshow("Mask", mask)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close windows
cap.release()
cv2.destroyAllWindows()
