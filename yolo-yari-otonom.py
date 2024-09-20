from ultralytics import YOLO
import cv2
import numpy as np
import math

stream_url = "rtsp://192.168.2.2:8554/video_stream__dev_video2"

cap = cv2.VideoCapture("video.mkv")
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

cap.set(3, 640)
cap.set(4, 480)

# Model
model = YOLO("model/best.pt")

# Object classes
classNames = ['Altigen', 'Besgen', 'Daire', 'Dikdortgen', 'Dort-Yaprakli-Yonca', 'Elips', 'Kare', 'Trombus', 'Ucgen', 'Yildiz']

flag_fps = 0
prev_result = None
prev_detections = []  # To store previous bounding boxes and class names

while True:
    success, img = cap.read()
    img = cv2.resize(img, (640, 480))
    if not success:
        break

    # Convert image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the HSV range for the color yellow
    lower_yellow = np.array([30, 150, 180])
    upper_yellow = np.array([60, 230, 230])

    # Create a mask for yellow color
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Perform morphological operations to remove noise and fill holes
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Create a white background
    white_background = np.ones_like(img, dtype=np.uint8) * 255

    # Apply the mask: keep yellow areas, and fill other areas with white
    pure_yellow = (0, 255, 255)  # BGR for yellow
    yellow_image = np.full_like(img, pure_yellow)

    # Apply the mask to the yellow image
    img_yellow = cv2.bitwise_and(yellow_image, yellow_image, mask=mask)
    img_white = cv2.bitwise_or(white_background, white_background, mask=~mask)

    # Combine yellow areas with the white background
    img_result = cv2.add(img_yellow, img_white)

    # Object detection every x frames
    if flag_fps % 15 == 0:
        gray_img = cv2.cvtColor(img_result, cv2.COLOR_BGR2GRAY)
        white_threshold = 0.98
        total_pixels = gray_img.size
        white_pixels = np.sum(gray_img > 240)
        white_percentage = white_pixels / total_pixels

        if not white_percentage > white_threshold or flag_fps == 0:
            results = model(img_result, stream=True)

        prev_result = results  # Store the latest results
        prev_detections = []  # Clear previous detections
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
                cls = int(box.cls[0])  # Class ID
                confidence = round(float(box.conf[0]), 2)  # Confidence score

                # Store the current detection details
                prev_detections.append((x1, y1, x2, y2, classNames[cls], confidence))

    # Draw the previous detection results
    for detection in prev_detections:
        x1, y1, x2, y2, class_name, confidence = detection

        # Draw bounding box and text on the original image
        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
        cv2.putText(img, f"{class_name}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

    flag_fps += 1

    # Show the result with text on the original webcam feed and mask
    cv2.imshow('Webcam', img_result)
    cv2.imshow('Normal Webcam Feed', img)

    # Exit on pressing 'q'
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
