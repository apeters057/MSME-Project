import sys
import cv2
import numpy as np

# Function to find LaneCenter Similar to 5LaneCenterCalc
# Adjust paths as needed 
sys.path.append('/home/apeters/project/env/src/Functions')
sys.path.append('/home/apeters/project/env/src')
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')
sys.path.append('/home/apeters/project/env/src/robot-hat')

def nothing(x):
    pass

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

# Main function to calculate lane center
def calculate_lane_center(frame):
    # Get HSV and Canny threshold values
    lower_green = np.array([29, 70, 135])
    upper_green = np.array([69, 255, 255])
    lower_canny = 50
    upper_canny = 150

    # Convert frame to HSV and apply mask
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, lower_green, upper_green)
    mask = cv2.GaussianBlur(mask, (13, 13), 0)

    # Apply Canny edge detection on the mask
    edges = cv2.Canny(mask, lower_canny, upper_canny)
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours for left (dashed) and right (solid) lane lines
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
            if contour_area > 20:  # Tune as needed for solid line
                right_contours.append(contour)

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
        right_contour = max(right_contours, key=cv2.contourArea)
        left_y_values = [point[1] for point in left_midpoints]
        right_midpoints = segment_contour(right_contour, left_y_values)

        # Calculate lane centers
        lane_centers = []
        right_corresponding_points = []
        used_right_points = set()

        for left_point in left_midpoints:
            # Find the closest right midpoint to each left midpoint at the same y-level
            same_level_right = [right for right in right_midpoints if abs(right[1] - left_point[1]) < 30 and right not in used_right_points]
            
            if same_level_right:
                closest_right = min(same_level_right, key=lambda right_point: abs(right_point[0] - left_point[0]))
                right_corresponding_points.append(closest_right)
                used_right_points.add(closest_right)
                center_x = (left_point[0] + closest_right[0]) // 2
                center_y = (left_point[1] + closest_right[1]) // 2
                lane_centers.append((center_x, center_y))

        # Calculate overall lane center
        if lane_centers:
            closest_lane_center = max(lane_centers, key=lambda point: point[1])
            furthest_lane_center = min(lane_centers, key=lambda point: point[1])

            overall_center_x = (closest_lane_center[0] + furthest_lane_center[0]) // 2
            overall_center_y = (closest_lane_center[1] + furthest_lane_center[1]) // 2

            # Draw for visual debugging (optional)
            for left_point in left_midpoints:
                cv2.circle(frame, left_point, 5, (255, 0, 0), -1)  # Blue for left dashed midpoint
            for right_point in right_corresponding_points:
                cv2.circle(frame, right_point, 5, (0, 0, 255), -1)  # Red for right solid midpoint
            for center in lane_centers:
                cv2.circle(frame, center, 5, (0, 255, 255), -1)  # Yellow for lane centers
            cv2.circle(frame, (overall_center_x, overall_center_y), 7, (0, 255, 255), -1)  # Larger yellow dot for overall center
            for i in range(1, len(lane_centers)):
                cv2.line(frame, lane_centers[i - 1], lane_centers[i], (0, 255, 0), 2)  # Green line for trajectory

            return (overall_center_x, overall_center_y), frame

    return None, frame  # Return None if no lane center found
# 
# # Example usage
# cap = cv2.VideoCapture(0)
# if __name__ == "__main__":
#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             break
# 
#         # Call the lane center calculation function
#         lane_center, processed_frame = calculate_lane_center(frame)
# 
#         # Display the processed frame with lane centers
#         cv2.imshow("Lane Center Calculation", processed_frame)
# 
#         # Use lane_center for PID control logic here, if applicable
#         if lane_center:
#             print("Overall Lane Center:", lane_center)
# 
#         # Press 'q' to quit
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
# 
#     cap.release()
#     cv2.destroyAllWindows()
