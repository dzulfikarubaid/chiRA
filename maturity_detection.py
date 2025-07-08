import numpy as np
import cv2
from ultralytics import YOLO

def detect_maturity(b, g, r):
    ripeness_level = "NaN"
    text_color = (255, 255, 255)
    bbox_and_bg_color = (0, 0, 0)
    if 0 < r - g < 25 and 5 < g - b < 25:
        ripeness_level = 'NaN'
    elif 30 > r > 0 and 30 > g > 0 :
        ripeness_level = 'NaN'
    elif g > b and (r - g) / 100 < 0.4 :
        ripeness_level = "Mentah"
        bbox_and_bg_color = (0, 205, 152)
        
    elif r > 80 and g > 80 and b < 50:
        ripeness_level = "Setengah"
        bbox_and_bg_color = (0, 101, 255)

    elif r > g and r > b and r > 75 :
        ripeness_level = "Matang"
        bbox_and_bg_color = (22, 22, 152)
    elif 200 > r > 100 or 200 > g > 100 :
        ripeness_level = '?'
      
    return ripeness_level, text_color, bbox_and_bg_color


def get_display_styling_params(img_width, img_height):
    reference_width = 1280
    scale_factor = img_width / reference_width
    scale_factor = max(0.5, min(scale_factor, 2.0))

    base_font_scale = 0.7
    base_font_thickness = 2
    base_line_thickness = 4
    padding_horizontal = int(10 * scale_factor)

    font_scale = base_font_scale * scale_factor
    font_thickness = max(1, int(base_font_thickness * scale_factor))
    line_thickness = max(2, int(base_line_thickness * scale_factor))

    return {
        'font_scale': font_scale,
        'font_thickness': font_thickness,
        'line_thickness': line_thickness,
        'padding_horizontal': padding_horizontal,
        'scale_factor': scale_factor
    }

def get_chili_maturity(cropped_chili_image):
    if cropped_chili_image is None or cropped_chili_image.size == 0:
        print("Warning: Input cropped_chili_image is empty or invalid.")
        return 'NaN', (255, 255, 255), (0, 0, 0)

    h, w, _ = cropped_chili_image.shape
    start_h, end_h = int(h * 0.47), int(h * 0.53)
    start_w, end_w = int(w * 0.47), int(w * 0.53)

    if start_h >= end_h: start_h = end_h - 1
    if start_w >= end_w: start_w = end_w - 1
    if start_h < 0: start_h = 0
    if start_w < 0: start_w = 0
    if end_h > h: end_h = h
    if end_w > w: end_w = w

    center_region = cropped_chili_image[start_h:end_h, start_w:end_w]
    
    if center_region.size > 0:
        b_avg, g_avg, r_avg = np.mean(center_region, axis=(0,1))
    else:
        print("Warning: Center region is empty. Cannot determine maturity.")
        return 'NaN', (255, 255, 255), (0, 0, 0)
    
    ripeness_level, text_color, bbox_and_bg_color = detect_maturity(b=b_avg, g=g_avg, r=r_avg)

    return ripeness_level, text_color, bbox_and_bg_color


def process_and_draw_detection(
    display_frame,
    original_frame,
    bbox_coords,
    confidence,
    styling_params
):
    x1, y1, x2, y2 = map(int, bbox_coords)

    x1 = max(0, x1)
    y1 = max(0, y1)
    x2 = min(original_frame.shape[1], x2)
    y2 = min(original_frame.shape[0], y2)

    if (x2 - x1) <= 0 or (y2 - y1) <= 0:
        print(f"Skipping invalid bounding box: ({x1},{y1}) to ({x2},{y2})")
        return

    cropped_chili = original_frame[y1:y2, x1:x2]

    ripeness_level, text_color, bbox_and_bg_color = get_chili_maturity(cropped_chili)

    if ripeness_level != 'NaN':
        font_scale = styling_params['font_scale']
        font_thickness = styling_params['font_thickness']
        line_thickness = styling_params['line_thickness']
        padding_horizontal = styling_params['padding_horizontal']
        scale_factor = styling_params['scale_factor']

        cv2.rectangle(display_frame, (x1, y1), (x2, y2), bbox_and_bg_color, line_thickness)

        text = f"{ripeness_level} ({confidence:.2f})"
        font = cv2.FONT_HERSHEY_SIMPLEX

        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, font_thickness)

        text_x = x1
        text_y = y1 - int(10 * scale_factor)

        if text_y < text_height + int(5 * scale_factor):
            text_y = y2 + text_height + baseline + int(5 * scale_factor)

        bg_start_x = max(0, text_x - padding_horizontal)
        bg_end_x = min(display_frame.shape[1], text_x + text_width + padding_horizontal)
        bg_start_y = max(0, text_y - text_height - baseline - int(5 * scale_factor))
        bg_end_y = min(display_frame.shape[0], text_y + baseline + int(5 * scale_factor))

        h_crop, w_crop, _ = cropped_chili.shape
        center_x1_abs = x1 + int(w_crop * 0.47)
        center_y1_abs = y1 + int(h_crop * 0.47)
        center_x2_abs = x1 + int(w_crop * 0.53)
        center_y2_abs = y1 + int(h_crop * 0.53)
        cv2.rectangle(display_frame, (center_x1_abs, center_y1_abs), (center_x2_abs, center_y2_abs), bbox_and_bg_color, line_thickness)

        cv2.rectangle(display_frame, (bg_start_x, bg_start_y), (bg_end_x, bg_end_y), bbox_and_bg_color, -1)
        cv2.putText(display_frame, text, (text_x, text_y), font, font_scale, text_color, font_thickness)