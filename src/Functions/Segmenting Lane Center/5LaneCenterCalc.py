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

# Function to segment contour based on y-values
def segment_contour(contour, y_values, segment_height=20):
    segmented_midpoints = []
    for y in y_values:
        segment_points = [point[0] for point in contour if y - segment_height <= point[0][1] <= y + segment_height]
        if segment_points:
            avg_x = int(np.mean([pt[0] for pt in segment_points]))
            avg_y = int(np.mean([pt[1] for pt in segment_points]))
            segmented_midpoints.append((avg_x, avg_y))
    return segmented_midpoints

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
    mask = cv2.GaussianBlur(mask, (13, 13), 0)

    #### Part 2: Edge Detection and Contour Detection ####
    # Apply Canny edge detection on the mask
    edges = cv2.Canny(mask, lc, uc)
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #### Part 3: Filtering Left (Dashed) and Right (Solid) Contours ####
    frame_height, frame_width = frame.shape[:2]
    left_contours = []
    right_contours = []

    for contour in contours:
        M = cv2.moments(contour)
        if M['m00'] == 0:
            continue

        cx = int(M['m10'] / M['m00'])  # X-coordinate of the contour's centroid
        cy = int(M['m01'] / M['m00'])  # Y-coordinate of the contour's centroid
        contour_area = cv2.contourArea(contour)

        # Classify as left (dashed) or right (solid) based on area and x-coordinates
        if cx < frame_width // 2:  # Left side (dashed lines)
            if 0 < contour_area < 1000:  # Tune as needed for dashed line segments
                left_contours.append((contour, (cx, cy)))  # Store contour and centroid
        else:  # Right side (solid line)
            if contour_area > 40:  # Tune as needed for solid line
                right_contours.append(contour)

    # Draw filtered contours for visual reference
    filtered_frame = frame.copy()
    for contour, _ in left_contours:
        cv2.drawContours(filtered_frame, [contour], -1, (255, 0, 0), 2)  # Blue for left (dashed)
    for contour in right_contours:
        cv2.drawContours(filtered_frame, [contour], -1, (0, 0, 255), 2)  # Red for right (solid)

    #### Part 4: Calculate Left Midpoints and Segment Right Contour ####
    # Process the largest right contour for midpoints
    if right_contours:
        right_contour = max(right_contours, key=cv2.contourArea)

    # Calculate midpoints of the left (dashed) line contours
    left_midpoints = []
    for contour, _ in left_contours:
        M_left = cv2.moments(contour)
        if M_left['m00'] != 0:
            cx_left = int(M_left['m10'] / M_left['m00'])
            cy_left = int(M_left['m01'] / M_left['m00'])
            left_midpoints.append((cx_left, cy_left))

    # Segment the right contour based on the y-values of the left midpoints
    if right_contours:
        left_y_values = [point[1] for point in left_midpoints]
        right_midpoints = segment_contour(right_contour, left_y_values)

        #### Part 5: Calculate Lane Centers ####
        lane_centers = []
        right_corresponding_points = []
        used_right_points = set()

        for left_point in left_midpoints:
            # Find the closest right midpoint to each left midpoint at the same y-level... adjust for lane segmentation 
            same_level_right = [right for right in right_midpoints if abs(right[1] - left_point[1]) < 30 and right not in used_right_points]
            
            if same_level_right:
                closest_right = min(same_level_right, key=lambda right_point: abs(right_point[0] - left_point[0]))
                right_corresponding_points.append(closest_right)
                used_right_points.add(closest_right)  # Mark this right point as used
                center_x = (left_point[0] + closest_right[0]) // 2
                center_y = (left_point[1] + closest_right[1]) // 2
                lane_centers.append((center_x, center_y))

                # Draw points and lines on filtered frame
                cv2.circle(filtered_frame, left_point, 5, (255, 0, 0), -1)  # Blue for left dashed midpoint
                cv2.circle(filtered_frame, closest_right, 5, (0, 0, 255), -1)  # Red for right solid midpoint
                cv2.circle(filtered_frame, (center_x, center_y), 5, (0, 255, 255), -1)  # Yellow for lane center
                cv2.line(filtered_frame, left_point, closest_right, (255, 0, 255), 2)  # Pink line between left and right points

        # Draw trajectory line connecting lane centers
        for i in range(1, len(lane_centers)):
            cv2.line(filtered_frame, lane_centers[i - 1], lane_centers[i], (0, 255, 0), 2)  # Green line for trajectory

        #### Calculate Overall Average Lane Center ####
        if lane_centers:
            # Find the closest and furthest lane centers by y-coordinate
            closest_lane_center = max(lane_centers, key=lambda point: point[1])
            furthest_lane_center = min(lane_centers, key=lambda point: point[1])

            # Calculate the average x and y values for the overall lane center
            overall_center_x = (closest_lane_center[0] + furthest_lane_center[0]) // 2
            overall_center_y = (closest_lane_center[1] + furthest_lane_center[1]) // 2

            # Draw the overall lane center as a larger yellow dot
            cv2.circle(filtered_frame, (overall_center_x, overall_center_y), 7, (0, 255, 255), -1)

    #### Display All Stages ####
    # Display original and processed frames
    cv2.imshow("Original", frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("Edges", edges)
    cv2.imshow("Filtered Contours and Lane Center Calculation", filtered_frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release video capture and close all windows
cap.release()
cv2.destroyAllWindows()
