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

# Perspective Transformation Function
def warp_perspective(frame):
    frame_height, frame_width = frame.shape[:2]

    # Static source points for lane boundaries
    src = np.float32([[0, 350],               # Bottom-left
                      [630, 350],             # Bottom-right
                      [150, 150],             # Top-left
                      [465, 150]])            # Top-right

    # Destination points to create a rectangular bird's-eye view
    dst = np.float32([[100, frame_height],              # Bottom-left in destination
                      [frame_width - 100, frame_height], # Bottom-right in destination
                      [100, 0],                          # Top-left in destination
                      [frame_width - 100, 0]])           # Top-right in destination

    # Perspective transformation
    M = cv2.getPerspectiveTransform(src, dst)
    warped_frame = cv2.warpPerspective(frame, M, (frame_width, frame_height))
    return warped_frame

while True:
    ret, frame = cap.read()
    if not ret:
        break

    #### Part 1: Mask Creation ####
    # Get HSV and Canny trackbar values
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
    mask = cv2.GaussianBlur(mask, (5, 5), 0)  # Set to (5, 5) for stability

    #### Part 2: Perspective Transformation on Mask ####
    # Apply perspective transformation on the mask to get a top-down view
    warped_mask = warp_perspective(mask)
    
    #### Part 3: Edge Detection and Contour Detection on Warped Frame ####
    # Apply Canny edge detection on the warped mask
    edges = cv2.Canny(warped_mask, lc, uc)
    
    # Find contours on the warped mask
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Initialize lists for left and right contours
    frame_height, frame_width = warped_mask.shape[:2]
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
            if 0 < contour_area < 1000:  # Adjust as needed for dashed line segments
                left_contours.append(contour)
        else:  # Right side (solid line)
            if contour_area > 50:  # Adjust as needed for solid line
                right_contours.append(contour)

    # Draw filtered left and right contours on the warped frame
    warped_contour_frame = cv2.cvtColor(warped_mask, cv2.COLOR_GRAY2BGR)  # Convert to BGR for contour display
    cv2.drawContours(warped_contour_frame, left_contours, -1, (255, 0, 0), 2)  # Blue for left (dashed)
    cv2.drawContours(warped_contour_frame, right_contours, -1, (0, 0, 255), 2)  # Red for right (solid)

    #### Part 4: Lane Center Calculation on Warped Frame ####
    # Calculate the average center of the left and right contours to find lane center
    lane_centers = []
    left_points = []
    right_points = []

    # Collect midpoints of each left and right contour
    for contour in left_contours:
        M = cv2.moments(contour)
        if M['m00'] != 0:
            cx_left = int(M['m10'] / M['m00'])
            cy_left = int(M['m01'] / M['m00'])
            left_points.append((cx_left, cy_left))

    for contour in right_contours:
        M = cv2.moments(contour)
        if M['m00'] != 0:
            cx_right = int(M['m10'] / M['m00'])
            cy_right = int(M['m01'] / M['m00'])
            right_points.append((cx_right, cy_right))

    # Calculate the lane center points based on left and right points
    if left_points and right_points:
        for i in range(min(len(left_points), len(right_points))):
            left_pt = left_points[i]
            right_pt = right_points[i]
            lane_center_x = (left_pt[0] + right_pt[0]) // 2
            lane_center_y = (left_pt[1] + right_pt[1]) // 2
            lane_centers.append((lane_center_x, lane_center_y))
            cv2.circle(warped_contour_frame, (lane_center_x, lane_center_y), 5, (0, 255, 255), -1)  # Yellow for lane center

    #### Display All Stages ####
    # Display original and processed frames
    cv2.imshow("Original", frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("Warped Perspective", warped_mask)
    cv2.imshow("Edges on Warped Frame", edges)
    cv2.imshow("Filtered Contours on Warped Frame", warped_contour_frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release video capture and close all windows
cap.release()
cv2.destroyAllWindows()
