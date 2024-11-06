import cv2
import numpy as np
import sys
sys.path.append('/home/apeters/project/env/src/Functions')
sys.path.append('/home/apeters/project/env/src')
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')
sys.path.append('/home/apeters/project/env/src/robot-hat')
# Capture video from the camera
import cv2
import numpy as np

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
cv2.createTrackbar("Lower Saturation", "Trackbars", 72, 255, nothing)
cv2.createTrackbar("Lower Value", "Trackbars", 75, 255, nothing)
cv2.createTrackbar("Upper Hue", "Trackbars", 69, 180, nothing)
cv2.createTrackbar("Upper Saturation", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("Upper Value", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("Lower Canny", "Trackbars", 50, 200, nothing)
cv2.createTrackbar("Upper Canny", "Trackbars", 150, 200, nothing)

#Better Value
#LH = 29
#LS =70
#LV = 135
#UH = 069
#US = 255
#UV = 255

# Capture video from the camera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
#Find Pixel Lane Color
    # Convert the frame to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
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
    # Create a mask to filter the green color
    mask = cv2.inRange(hsv_frame, lower_green, upper_green)
    # Perform bitwise-AND between the mask and the original frame
    res = cv2.bitwise_and(frame, frame, mask=mask)
    # Show the original frame, mask, and the result
    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", res)
    
#Lane Detection
    # Use Canny Edge Detection to find lane edges
    edges = cv2.Canny(mask, 0, 150)
    # Find contours of the green tape
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Draw contours on the original frame
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
    # Display the result
    
    cv2.imshow("Result", edges)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
