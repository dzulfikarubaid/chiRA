import cv2
from ultralytics import YOLO

import config

model = YOLO(config.MODEL_PATH)
img = cv2.imread('images/5.png')
results = model(img, conf=config.CONFIDENCE_THRESHOLD, device='cuda')
for result in results:
    result.show()