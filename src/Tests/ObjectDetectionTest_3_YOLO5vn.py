import cv2
import torch

# Load the YOLOv5n model with half precision
model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True).half()

# Start video capture (0 for the default camera)
cap = cv2.VideoCapture(0)

# Set the desired frame width and height
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Adjust as needed
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Adjust as needed

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Resize frame for faster processing
    frame_resized = cv2.resize(frame, (320, 240))  # Adjust size as needed

    # Perform detection
    results = model(frame_resized)

    # Render results on the original frame
    frame = results.render()[0]

    # Display the resulting frame
    cv2.imshow('Object Detection', frame)

    # Add a delay to control the frame rate
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and destroy windows
cap.release()
cv2.destroyAllWindows()