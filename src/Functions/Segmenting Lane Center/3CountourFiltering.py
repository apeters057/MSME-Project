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

# Create trackbars for adjusting HSV values and Canny thresholds
cv2.namedWindow("Trackbars")
cv2.createTrackbar("Lower Hue", "Trackbars", 29, 180, nothing)
cv2.createTrackbar("Lower Saturation", "Trackbars", 70, 255, nothing)
cv2.createTrackbar("Lower Value", "Trackbars", 135, 255, nothing)
cv2.createTrackbar("Upper Hue", "Trackbars", 69, 180, nothing)
cv2.createTrackbar("Upper Saturation", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("Upper Value", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("Lower Canny", "Trackbars", 50, 200, nothing)
cv2.createTrackbar("Upper Canny", "Trackbars", 150, 200, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        break
####   PART 1 ####
#Find Pixel Lane Color
    # Get current positions of the trackbars
    # Track HSV and Canny threshold values from the trackbars
    lh = cv2.getTrackbarPos("Lower Hue", "Trackbars")
    ls = cv2.getTrackbarPos("Lower Saturation", "Trackbars")
    lv = cv2.getTrackbarPos("Lower Value", "Trackbars")
    uh = cv2.getTrackbarPos("Upper Hue", "Trackbars")
    us = cv2.getTrackbarPos("Upper Saturation", "Trackbars")
    uv = cv2.getTrackbarPos("Upper Value", "Trackbars")
    lc = cv2.getTrackbarPos("Lower Canny", "Trackbars")
    uc = cv2.getTrackbarPos("Upper Canny", "Trackbars")

    # Define HSV range for green lane color
    lower_green = np.array([lh, ls, lv])
    upper_green = np.array([uh, us, uv])

    # Convert frame to HSV and apply mask
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, lower_green, upper_green)
    mask = cv2.GaussianBlur(mask, (13, 13), 0)
#### Part 2 ####
    ############# Parameters to Tune
    ############# Canny thresholds (50, 150) ---- Higher values will focus on stronger edges, while lower values will detect more details.
    # Edge detection
    edges = cv2.Canny(mask, lc, uc)
    # Find all contours
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Draw all contours for reference (without filtering)
    contour_frame = frame.copy()
    cv2.drawContours(contour_frame, contours, -1, (0, 255, 0), 2)
#### Part 3 ####
    ############# Parameters to Tune
     ############# LINES 81 & 85
    # Initialize frame dimensions and lists for left and right contours
    frame_height, frame_width = frame.shape[:2]
    left_contours = []
    right_contours = []

    # Filter contours for left (dashed) and right (solid) lanes based on area and position
    for contour in contours:
        M = cv2.moments(contour)
        if M['m00'] == 0:
            continue

        cx = int(M['m10'] / M['m00'])  # X-coordinate of the contour's centroid
        contour_area = cv2.contourArea(contour)

        # Classify as left or right based on area and x-coordinates
        if cx < frame_width // 2:  # Left side (dashed lines)
            if 0 < contour_area < 1000:  # Start with a narrower range to reduce noise.. #TUNE
                left_contours.append(contour)
        else:  # Right side (solid line)
            if contour_area > 90:  # Set a minimum area to reduce small, noisy contours.. #TUNE
                right_contours.append(contour)

    # Draw filtered left and right contours for debugging
    filtered_frame = frame.copy()
    cv2.drawContours(filtered_frame, left_contours, -1, (255, 0, 0), 2)  # Blue for left (dashed)
    cv2.drawContours(filtered_frame, right_contours, -1, (0, 0, 255), 2)  # Red for right (solid)

    # Display all stages for comparison
    #Part 1
    cv2.imshow("Original", frame)
    cv2.imshow("Mask", mask)
    #Part 2
    cv2.imshow("Edges", edges)
    cv2.imshow("Contours", contour_frame)
    #Part3
    cv2.imshow("Filtered Contours", filtered_frame)
    

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
