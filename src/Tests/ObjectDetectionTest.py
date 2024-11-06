import sys
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')

import cv2
import numpy as np
import tflite_runtime.interpreter as tflite


# Load the TFLite model and allocate tensors
interpreter = tflite.Interpreter(model_path='/home/apeters/project/env/lib/python3.9/site-packages/detect.tflite')
interpreter.allocate_tensors()

# Get input and output tensors.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Load the COCO labels
with open('/home/apeters/project/env/lib/python3.9/site-packages/labelmap.txt', 'r') as f:
    labels = [line.strip() for line in f.readlines()]
    
# Access the camera (0 is usually the default camera on a Raspberry Pi)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
# Pre-process the frame for the model
    input_size = input_details[0]['shape'][1:3]
    img_resized = cv2.resize(frame, input_size)
    img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
    input_data = np.expand_dims(img_rgb, axis=0)



# Perform the inference
    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()


# Get output data
    boxes = interpreter.get_tensor(output_details[0]['index'])[0]  # Bounding box coordinates
    classes = interpreter.get_tensor(output_details[1]['index'])[0]  # Class index
    scores = interpreter.get_tensor(output_details[2]['index'])[0]  # Confidence scores


        # Draw the results of the detection
    for i in range(len(scores)):
        if scores[i] > 0.5:  # Only consider detections with confidence > 0.5
            ymin, xmin, ymax, xmax = boxes[i]
            class_id = int(classes[i])
            label = labels[class_id]
            confidence = scores[i]

        # Convert the bounding box from relative coordinates to image coordinates
            imH, imW, _ = frame.shape
            xmin = int(xmin * imW)
            xmax = int(xmax * imW)
            ymin = int(ymin * imH)
            ymax = int(ymax * imH)

                # Draw the bounding box and label on the frame
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            cv2.putText(frame, f'{label} {confidence:.2f}', (xmin, ymin - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the output frame
        cv2.imshow('Object Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release the video capture object and close display window
cap.release()
cv2.destroyAllWindows()


