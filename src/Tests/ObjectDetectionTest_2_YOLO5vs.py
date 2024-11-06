import cv2
import torch
import time
import threading  # Use the threading module

# Threaded video stream class to capture frames faster
class VideoStream:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.grabbed, self.frame = self.cap.read()
        self.started = False
        self.read_lock = threading.Lock()  # Use threading.Lock()

    def start(self):
        if self.started:
            print("Video stream already started.")
            return None
        self.started = True
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.start()
        return self

    def update(self):
        while self.started:
            grabbed, frame = self.cap.read()
            with self.read_lock:
                self.grabbed = grabbed
                self.frame = frame

    def read(self):
        with self.read_lock:
            frame = self.frame.copy()
        return frame

    def stop(self):
        self.started = False
        self.thread.join()
        self.cap.release()

# Load YOLOv5 model (nano version for speed)
model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True).eval()

# Start the threaded video stream
video_stream = VideoStream().start()

# Set frame skipping factor
frame_skip = 3  # Process every frame, adjust if necessary
frame_count = 0

# Measure time to calculate FPS
fps_counter = 0
start_time = time.time()

while True:
    # Read the latest frame from the stream
    frame = video_stream.read()
    
    # Skip frame processing if needed
    if frame_count % frame_skip == 0:
        # Start measuring detection time
        detection_start_time = time.time()

        # Resize the frame for faster processing
        frame_resized = cv2.resize(frame, (320, 240))

        # Perform object detection
        results = model(frame_resized)
        
        detection_time = time.time() - detection_start_time  # Calculate detection time

        # Print detection results
        detections = results.xyxy[0]
        for *box, conf, cls in detections.tolist():
            label = model.names[int(cls)]
            print(f"Detected: {label}, Confidence: {conf:.2f}, Box: {box}")
        
        # Calculate FPS for real-world performance
        fps_counter += 1
        elapsed_time = time.time() - start_time
        if elapsed_time > 1.0:  # Update FPS every second
            fps = fps_counter / elapsed_time
            print(f"Real-World FPS: {fps:.2f}, Detection Time: {detection_time:.2f} seconds")
            fps_counter = 0
            start_time = time.time()

    frame_count += 1

    # Display the frame
    cv2.imshow('Object Detection', frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video stream and close windows
video_stream.stop()
cv2.destroyAllWindows()
