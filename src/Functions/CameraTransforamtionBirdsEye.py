## Birds Eye View Lane Calculation
import sys
import cv2
import numpy as np

sys.path.append('/home/apeters/project/env/src/Functions')
sys.path.append('/home/apeters/project/env/src')
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')
sys.path.append('/home/apeters/project/env/src/robot-hat')

# Capture video from the camera
cap = cv2.VideoCapture(0)
# Define the warp_perspective function
def warp_perspective(frame):
    frame_height, frame_width = frame.shape[:2]

    # Source points (adjust these to match your lane's location in the frame)
    # Order: Bottom-left, Bottom-right, Top-right, Top-left
    src = np.float32([[100, frame_height],              # Bottom-left
                      [frame_width - 100, frame_height],  # Bottom-right
                      [400, 100],                         # Top-right
                      [frame_width - 400, 100]          
                      ])                                    # Top-left

    # Destination points (define where the src points should map in the warped frame)
    # Order: Bottom-left, Bottom-right, Top-right, Top-left
    dst = np.float32([[200, frame_height],                # Bottom-left
                      [frame_width - 100, frame_height],  # Bottom-right
                      [frame_width - 100, 0],            # Top-right
                      [200, 0]])                         # Top-left

    # Get the perspective transformation matrix
    M = cv2.getPerspectiveTransform(src, dst)

    # Warp the perspective
    warped_frame = cv2.warpPerspective(frame, M, (frame_width, frame_height))

    return warped_frame


while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Apply the perspective warp
    warped_frame = warp_perspective(frame)

    # Display the original and warped images side by side for comparison
    cv2.imshow("Original Frame", frame)
    cv2.imshow("Warped Frame (Bird's Eye View)", warped_frame)

    # Wait for a key press and close the windows
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close windows
cap.release()
cv2.destroyAllWindows()
