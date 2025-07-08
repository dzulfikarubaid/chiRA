import cv2
import numpy as np
from ultralytics import YOLO
from maturity_detection import detect_maturity,  get_chili_maturity, get_display_styling_params, process_and_draw_detection

model = YOLO('models/cabaiRawitNew_4_119_64.pt', task='detect')
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    display_frame = frame.copy()
    img_height, img_width = frame.shape[:2] # Ambil dimensi dari frame saat ini

    # Dapatkan parameter styling untuk frame saat ini
    styling_params = get_display_styling_params(img_width, img_height)

    results = model.predict(frame, conf=0.6, device='cuda', verbose=False)

    for result in results:
        boxes = result.boxes.xyxy.cpu().numpy()
        confidences = result.boxes.conf.cpu().numpy()

        for i in range(len(boxes)):
            bbox_coords = boxes[i]
            confidence = confidences[i]
            
            process_and_draw_detection(
                display_frame,
                frame, # Kirimkan frame asli untuk cropping
                bbox_coords,
                confidence,
                styling_params
            )

    cv2.imshow("Real-Time Chili Maturity Detection", display_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()