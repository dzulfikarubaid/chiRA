import cv2
import numpy as np
from ultralytics import YOLO
from maturity_detection import detect_maturity,  get_chili_maturity, get_display_styling_params, process_and_draw_detection


img_name = '1.webp'
img_path = f'images/test/{img_name}'
model = YOLO('models/cabaiRawitNew_4_119_64.pt', task='detect')

img = cv2.imread(img_path)
if img is None:
    print(f"Error: Gambar tidak ditemukan di {img_path}")
    exit()

display_img = img.copy()
img_height, img_width = img.shape[:2]

styling_params = get_display_styling_params(img_width, img_height)

results = model.predict(img_path, conf=0.6, device='cuda', verbose=True)

for result in results:
    boxes = result.boxes.xyxy.cpu().numpy()
    confidences = result.boxes.conf.cpu().numpy()

    for i in range(len(boxes)):
        bbox_coords = boxes[i]
        confidence = confidences[i]
        
        process_and_draw_detection(
            display_img,
            img, 
            bbox_coords,
            confidence,
            styling_params
        )


max_display_width = 1200
max_display_height = 800

if img_width > max_display_width or img_height > max_display_height:
    width_ratio = max_display_width / img_width
    height_ratio = max_display_height / img_height

    display_scale = min(width_ratio, height_ratio)

    new_width = int(img_width * display_scale)
    new_height = int(img_height * display_scale)

    display_img = cv2.resize(display_img, (new_width, new_height), interpolation=cv2.INTER_AREA)
else:
    display_img = display_img.copy()

cv2.imshow("Deteksi Kematangan Cabai", display_img)
cv2.imwrite(img_name, display_img)
cv2.waitKey(0)
cv2.destroyAllWindows()


