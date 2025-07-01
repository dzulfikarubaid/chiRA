from ultralytics import YOLO

import config

model = YOLO(config.MODEL_PATH)
results = model.val()
print(results)
