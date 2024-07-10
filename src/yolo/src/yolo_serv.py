import torch
import torchvision.transforms as transforms
import cv2
import numpy as np

# Step 1: Load YOLOv8 Model
model_path = 'best.pt'
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = torch.load(model_path, map_location=device)['model']
model.eval()

# Step 2: Preprocess the Image
def preprocess_image(frame):
    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Resize((416, 416)), 
    ])
    return transform(frame).unsqueeze(0)

# Step 3: Run Inference
def run_inference(frame):
    with torch.no_grad():
        frame_tensor = preprocess_image(frame).to(device)
        outputs = model(frame_tensor)
        # Process YOLOv8 outputs here to extract x, y, width, height, class, confidence
        detections = postprocess_yolov8_outputs(outputs)
        return detections

# Step 4: Post-process the Output
def postprocess_yolov8_outputs(outputs):
    # Example post-processing code (adjust according to your model's output format)
    boxes = outputs['boxes'].cpu().numpy()
    classes = outputs['classes'].cpu().numpy()
    scores = outputs['scores'].cpu().numpy()

    detections = []
    for box, cls, score in zip(boxes, classes, scores):
        x1, y1, x2, y2 = box.astype(np.int)
        x = (x1 + x2) / 2  # Horizontal center
        y = (y1 + y2) / 2  # Vertical center
        width = x2 - x1     # Width of bounding box
        height = y2 - y1    # Height of bounding box
        class_label = cls   # Class label
        confidence = score  # Confidence score

        detections.append({
            'x': x,
            'y': y,
            'width': width,
            'height': height,
            'class': class_label,
            'confidence': confidence
        })

    return detections

# Example of processing an image frame
def process_image_frame(frame):
    detections = run_inference(frame)
    return detections
    for detection in detections:
        print(f"Class: {detection['class']}, Confidence: {detection['confidence']:.2f}")
        print(f"Center (x, y): ({detection['x']}, {detection['y']})")
        print(f"Width: {detection['width']}, Height: {detection['height']}")
        print("")


