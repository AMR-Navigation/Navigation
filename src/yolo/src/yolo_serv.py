from ultralytics import YOLO
import cv2
import base64
import numpy as np

# Load a pretrained YOLOv8 model
model = YOLO("yolov8.pt")

# List of class names
class_names = ["Chair", "People", "Robot", "Sofa", "Table"]

# Function to process and print detections
def process_image_frame(frame):
    # Run inference
    results = model(frame)

    detections = []
    
    for r in results:
        boxes = r.boxes
        for box in boxes:
            # Get bounding box coordinates, confidence, and class
            xywh = box.xywh.numpy()[0]
            confidence = box.conf.numpy()[0]
            cls = int(box.cls.numpy()[0])
            
            class_name = class_names[cls]
            x, y, width, height = xywh

            detection = {
                'class': class_name,
                'confidence': float(confidence),
                'x': float(x),
                'y': float(y),
                'width': float(width),
                'height': float(height)
            }
            detections.append(detection)

    return detections


"""# The rest of the code is for testing purposes only (inserting a jpg image)
with open("person.jpg", "rb") as image_file:
    base64_image = base64.b64encode(image_file.read()).decode('utf-8')

# Decode base64 image string to numpy array
decoded_image = base64.b64decode(base64_image)
nparr = np.frombuffer(decoded_image, np.uint8)
frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# Process the decoded image frame
process_image_frame(frame)"""
