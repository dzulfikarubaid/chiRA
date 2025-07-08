import numpy as np
import serial
import time
import cv2
from ultralytics import YOLO
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import requests
from inverse_kinematics import calculate_angles
import datetime
import config
from maturity_detection import detect_maturity, get_chili_maturity

cred = credentials.Certificate(config.FIREBASE_CREDENTIAL_PATH)
firebase_admin.initialize_app(cred, {
    'databaseURL': config.FIREBASE_DATABASE_URL
})
firebase_ref = db.reference('robot_status')
firebase_stats_ref = db.reference('statistics')
firebase_daily_stats_ref = db.reference('daily_chili_picks')
firebase_monthly_stats_ref = db.reference('monthly_chili_picks')
firebase_hourly_stats_ref = db.reference('hourly_chili_picks')

def send_status_to_firebase(status, angles=None, ee_angle=None, detected_object_position_cm=None, error=None, picked=False):
    status_data = {
        'status': status,
        'timestamp': time.time()
    }
    if angles:
        status_data['sudut1'] = angles[0]
        status_data['sudut2'] = angles[1]
        status_data['sudut3'] = angles[2]
    if ee_angle is not None:
        status_data['ee_angle'] = ee_angle
    if detected_object_position_cm:
        status_data['detected_object_cm'] = {
            'x': float(detected_object_position_cm[0]),
            'y': float(detected_object_position_cm[1]),
            'z': float(detected_object_position_cm[2])
        }
    if error:
        status_data['error'] = error
    if picked:
        status_data['picked'] = picked
    try:
        firebase_ref.set(status_data)
    except Exception as e:
        print(f"Failed to send status to Firebase: {e}")

def send_chili_count_to_firebase(count):
    try:
        firebase_stats_ref.update({'chili_picked_count': count, 'last_update_count': time.time()})
        now = datetime.datetime.now()
        date_key_hourly = now.strftime('%Y-%m-%d')
        hour_key = now.strftime('%H')
        current_hourly_count_ref = firebase_hourly_stats_ref.child(date_key_hourly).child(hour_key).child('count')
        current_hourly_count = current_hourly_count_ref.get() or 0
        current_hourly_count_ref.set(current_hourly_count + 1)
        date_key_daily = now.strftime('%Y-%m')
        day_key = now.strftime('%d')
        current_daily_count_ref = firebase_daily_stats_ref.child(date_key_daily).child(day_key).child('count')
        current_daily_count = current_daily_count_ref.get() or 0
        current_daily_count_ref.set(current_daily_count + 1)
        year_key_monthly = now.strftime('%Y')
        month_key = now.strftime('%m')
        current_monthly_count_ref = firebase_monthly_stats_ref.child(year_key_monthly).child(month_key).child('count')
        current_monthly_count = current_monthly_count_ref.get() or 0
        current_monthly_count_ref.set(current_monthly_count + 1)
    except Exception as e:
        print(f"Failed to send chili count to Firebase: {e}")

def send_total_attempts_to_firebase(count):
    try:
        firebase_stats_ref.update({'total_picking_attempts': count, 'last_update_attempts': time.time()})
    except Exception as e:
        print(f"Failed to send total picking attempts to Firebase: {e}")

def clamp_angle(angle, min_angle, max_angle):
    return max(min(angle, max_angle), min_angle)

def send_serial_command(ser, command_type, angles, ee_angle, move_time=1000):
    theta1, theta2, theta3 = angles
    data_to_send = f"<{command_type},{ee_angle:.2f},{theta1:.2f},{theta2:.2f},{theta3:.2f},{move_time},{move_time},{move_time},{move_time}>"
    if ser:
        ser.write(data_to_send.encode())
    send_status_to_firebase("SENDING_COMMAND", angles=(theta1, theta2, theta3), ee_angle=ee_angle)
    time.sleep(0.1)

def calculate_distance(point_left, point_right, Q):
    disparity = point_right[0] - point_left[0]
    if disparity <= 0:
        return None
    pw = np.array([point_right[0], point_right[1], disparity, 1], dtype=np.float32).reshape(-1, 1)
    homogeneous_3D = Q @ pw
    X = homogeneous_3D[0] / homogeneous_3D[3]
    Y = homogeneous_3D[1] / homogeneous_3D[3]
    Z = homogeneous_3D[2] / homogeneous_3D[3]
    return X[0], Y[0], Z[0]

def adjust_saturation_brightness(image, saturation_factor=1.5, brightness_factor=1.2):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    s = np.clip(s * saturation_factor, 0, 255).astype(np.uint8)
    v = np.clip(v * brightness_factor, 0, 255).astype(np.uint8)
    hsv_merged = cv2.merge([h, s, v])
    adjusted_image = cv2.cvtColor(hsv_merged, cv2.COLOR_HSV2BGR)
    return adjusted_image

def find_matching_objects(resultsL, resultsR):
    boxesL = resultsL[0].boxes.xyxy.cpu().numpy() if resultsL and resultsL[0].boxes else np.array([])
    class_idsL = resultsL[0].boxes.cls.cpu().numpy() if resultsL and resultsL[0].boxes else np.array([])
    boxesR = resultsR[0].boxes.xyxy.cpu().numpy() if resultsR and resultsR[0].boxes else np.array([])
    class_idsR = resultsR[0].boxes.cls.cpu().numpy() if resultsR and resultsR[0].boxes else np.array([])
    matches = []
    used_right = [False] * len(boxesR)
    for i, boxL in enumerate(boxesL):
        center_y_L = (boxL[1] + boxL[3]) // 2
        class_id_L = class_idsL[i]
        best_match_idx = -1
        min_dy = float('inf')
        for j, boxR in enumerate(boxesR):
            if not used_right[j]:
                center_y_R = (boxR[1] + boxR[3]) // 2
                class_id_R = class_idsR[j]
                dy = abs(center_y_L - center_y_R)
                if dy < config.MATCH_THRESHOLD_Y and class_id_L == class_id_R:
                    if dy < min_dy:
                        min_dy = dy
                        best_match_idx = j
        if best_match_idx != -1:
            matches.append((i, best_match_idx))
            used_right[best_match_idx] = True
    return matches, boxesL, class_idsL, boxesR, class_idsR

if __name__ == "__main__":
    try:
        model = YOLO(config.MODEL_PATH, task="detect")
        calibration_data = np.load(config.CALIBRATION_FILE)
        if not all(k in calibration_data for k in ['map1_l', 'map2_l', 'map1_r', 'map2_r', 'Q']):
            raise KeyError("Kunci kalibrasi tidak lengkap (pastikan ada 'Q')")
        Q = calibration_data['Q']
    except Exception as e:
        send_status_to_firebase("ERROR", error=str(e))
        exit()

    cap_left = cv2.VideoCapture(0)
    cap_right = cv2.VideoCapture(1)

    if not (cap_left.isOpened() and cap_right.isOpened()):
        send_status_to_firebase("ERROR", error="Gagal membuka salah satu kamera")
        exit()

    try:
        ser = serial.Serial(config.SERIAL_PORT, config.BAUD_RATE, timeout=config.SERIAL_TIMEOUT)
        time.sleep(2)
    except serial.SerialException as e:
        send_status_to_firebase("ERROR", error=f"Gagal membuka port serial {config.SERIAL_PORT}: {e}")
        ser = None

    robot_state = "IDLE"
    object_has_been_picked = False
    picked_chili_count = 0
    total_picking_attempts = 0

    send_status_to_firebase(robot_state, config.INITIAL_ANGLES, config.INITIAL_EE_ANGLE)
    send_chili_count_to_firebase(picked_chili_count)
    send_total_attempts_to_firebase(total_picking_attempts)

    prev_frame_time = 0
    new_frame_time = 0
    last_object_detection_time = None
    robot_move_command_time = None
    angles_to_target = config.INITIAL_ANGLES

    while True:
        new_frame_time = time.time()
        
        retL, frameL = cap_left.read()
        retR, frame_R = cap_right.read()
        
        if not (retL and retR):
            send_status_to_firebase("ERROR", error="Gagal membaca frame dari kamera")
            continue
        
        if config.FILTER_IMAGE:
            frameL = adjust_saturation_brightness(frameL)
            frame_R = adjust_saturation_brightness(frame_R)

        rectifiedL = cv2.remap(frameL, calibration_data['map1_l'], calibration_data['map2_l'], cv2.INTER_LINEAR)
        rectifiedR = cv2.remap(frame_R, calibration_data['map1_r'], calibration_data['map2_r'], cv2.INTER_LINEAR)

        detection_start_time = time.time()
        resultsL = model(rectifiedL, verbose=False, conf=config.CONFIDENCE_THRESHOLD, device='cuda')
        resultsR = model(rectifiedR, verbose=False, conf=config.CONFIDENCE_THRESHOLD, device='cuda')
        detection_end_time = time.time()
        detection_latency = detection_end_time - detection_start_time

        matches, boxesL, class_idsL, boxesR, class_idsR = find_matching_objects(resultsL, resultsR)
        detected_object_position_m = None
        detected_object_position_cm = None

        annotated_rectifiedL = rectifiedL.copy()
        annotated_rectifiedR = rectifiedR.copy()

        for idx_L, idx_R in matches:
            boxL = boxesL[idx_L].astype(int)
            boxR = boxesR[idx_R].astype(int)
            x1L, y1L, x2L, y2L = boxL
            x1R, y1R, x2R, y2R = boxR
            center_L = ((x1L + x2L) // 2, (y1L + y2L) // 2)
            center_R = ((x1R + x2R) // 2, (y1R + y2R) // 2)
            distance_m = calculate_distance(center_L, center_R, Q)
            
            hL, wL = y2L - y1L, x2L - x1L
            start_hL = int(hL * 0.45)
            end_hL = int(hL * 0.55)
            start_wL = int(wL * 0.45)
            end_wL = int(wL * 0.55)
            
            hR, wR = y2R - y1R, x2R - x1R
            start_hR = int(hR * 0.45)
            end_hR = int(hR * 0.55)
            start_wR = int(wR * 0.45)
            end_wR = int(wR * 0.55)
            
            
            if start_hL >= end_hL or start_wL >= end_wL:
                center_region_L = rectifiedL[y1L:y2L, x1L:x2L]
            else:
                center_region_L = rectifiedL[y1L + start_hL : y1L + end_hL, x1L + start_wL : x1L + end_wL]
                
            if start_hR >= end_hR or start_wR >= end_wR:
                center_region_R = rectifiedR[y1R:y2R, x1R:x2R]
            else:
                center_region_R = rectifiedR[y1R + start_hR : y1R + end_hR, x1R + start_wR : x1R + end_wR]
            
            ripeness_levelL, text_colorL, bbox_and_bg_colorL = get_chili_maturity(center_region_L) 
            ripeness_levelR, text_colorR, bbox_and_bg_colorR = get_chili_maturity(center_region_R) 
            
            if ripeness_levelL != 'NaN' and ripeness_levelR != 'NaN' and ripeness_levelR == ripeness_levelL:
                
                center_box_x1L = x1L + start_wL
                center_box_y1L = y1L + start_hL
                center_box_x2L = x1L + end_wL
                center_box_y2L = y1L + end_hL
                
                center_box_x1R = x1R + start_wR
                center_box_y1R = y1R + start_hR
                center_box_x2R = x1R + end_wR
                center_box_y2R = y1R + end_hR
                
                cv2.rectangle(annotated_rectifiedL, (x1L, y1L), (x2L, y2L), bbox_and_bg_colorL, 2)
                cv2.rectangle(annotated_rectifiedR, (x1R, y1R), (x2R, y2R), bbox_and_bg_colorR, 2)
                
                cv2.rectangle(annotated_rectifiedL, (center_box_x1L, center_box_y1L), (center_box_x2L, center_box_y2L), bbox_and_bg_colorL, 1)
                cv2.rectangle(annotated_rectifiedR, (center_box_x1R, center_box_y1R), (center_box_x2R, center_box_y2R), bbox_and_bg_colorR, 1)

                if distance_m is not None:
                    X_m, Y_m, Z_m = distance_m
                    detected_object_position_m = (X_m, Y_m, Z_m)
                    robot_z_cm = -Z_m * 100 - config.Z_ADJUSTMENT
                    robot_x_cm = X_m * 100 + config.X_ADJUSTMENT
                    robot_y_cm = Y_m * 100 + config.Y_ADJUSTMENT
                    if(robot_z_cm <= 22):
                        robot_z_cm += 2
                        robot_y_cm += 3
                    elif(22 < robot_z_cm < 26.1):
                        robot_z_cm += 2
                        robot_y_cm += 2
                    
                    detected_object_position_cm = (robot_x_cm, robot_y_cm, robot_z_cm)
                    
                    if last_object_detection_time is None:
                        last_object_detection_time = time.time()

                    distance_text = f"X:{robot_z_cm:.2f} Y:{robot_x_cm:.2f} Z:{robot_y_cm:.2f} cm"
                    cv2.putText(annotated_rectifiedL, distance_text, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
                    
                    text_position = (10+5, 85+5) 
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.5
                    thickness = 2
                    
                    (text_width, text_height), baseline = cv2.getTextSize(ripeness_levelL, font, font_scale, thickness)
                    
                    rect_top_left = (text_position[0] - 5, text_position[1] - text_height - baseline) 
                    rect_bottom_right = (text_position[0] + text_width + 5, text_position[1] + baseline + 2)
                    
                    cv2.rectangle(annotated_rectifiedL, rect_top_left, rect_bottom_right, bbox_and_bg_colorL, -1)
                    cv2.putText(annotated_rectifiedL, ripeness_levelL, text_position, font, font_scale, text_colorL, thickness, cv2.LINE_AA)
                    
                    break
            else:
                break

        fps = 1 / (new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time
        fps_text = f"FPS: {int(fps)}"
        cv2.putText(annotated_rectifiedL, fps_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
        
        try:
            combined_rectified = cv2.hconcat([annotated_rectifiedL, annotated_rectifiedR])
            cv2.imshow("Stereo Rectified with Detection (Left | Right)", combined_rectified)
        except Exception as e:
            print(f"Error displaying image: {e}")

        current_loop_time = time.time()

        if ser is not None:
            if ser.in_waiting > 0:
                feedback = ser.readline().decode().strip()
                if feedback == "<PICKED>" and robot_state != 'IDLE':
                    object_has_been_picked = True
                    picked_chili_count += 1
                    send_chili_count_to_firebase(picked_chili_count)
                    print('[Chili has been picked]')
                    send_status_to_firebase("PICKED", picked=True)
                    if robot_state != "_AFTER_PICK":
                        send_serial_command(ser, "MOVE_ROBOT_ARM", config.INITIAL_ANGLES, config.GRAB_EE_ANGLE)
                        robot_state = "_AFTER_PICK"
                        send_status_to_firebase(robot_state, angles=config.INITIAL_ANGLES, ee_angle=config.GRAB_EE_ANGLE)
                        return_start_time = time.time()

            if robot_state == "IDLE":
                send_status_to_firebase(robot_state)
                # print(detected_object_position_cm)
                if detected_object_position_cm and not object_has_been_picked and ripeness_levelL != 'Mentah' and config.RUN_ROBOT:
                    obj_x_cm, obj_y_cm, obj_z_cm = detected_object_position_cm
                    send_status_to_firebase("OBJECT_DETECTED", detected_object_position_cm=[obj_x_cm, obj_y_cm, obj_z_cm])
                    try:
                        ik_start_time = time.time()
                        theta_1_servo, theta_2_servo, theta_3_servo, is_out_of_reach = calculate_angles(obj_z_cm, obj_x_cm, obj_y_cm)
                        ik_end_time = time.time()
                        ik_latency = ik_end_time - ik_start_time
                        
                        latency_text_ik = f"IK Latency: {ik_latency*1000:.2f}ms"
                        # cv2.putText(annotated_rectifiedL, latency_text_ik, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)

                        angles_to_target = [theta_1_servo, theta_2_servo, theta_3_servo]
                        
                        if is_out_of_reach:
                            send_status_to_firebase("ERROR", error="Objek tidak terjangkau")
                            robot_state = "IDLE"
                        elif np.isnan(angles_to_target).any():
                            send_status_to_firebase("ERROR", error="Sudut target tidak valid (NaN)")
                            robot_state = "IDLE"
                        else:
                            send_serial_command(ser, "MOVE_ROBOT_ARM", angles_to_target[:3], config.INITIAL_EE_ANGLE)
                            robot_state = "MOVING_TO_OBJECT"
                            print('=============================')
                            print('Move to: ', obj_z_cm, obj_x_cm, obj_y_cm, )
                            print('Servo angles: ', theta_1_servo, theta_2_servo, theta_3_servo, is_out_of_reach)
                            total_picking_attempts += 1
                            send_total_attempts_to_firebase(total_picking_attempts)
                            robot_move_command_time = time.time()
                            
                            if last_object_detection_time:
                                end_to_end_latency = robot_move_command_time - last_object_detection_time
                                latency_text_e2e = f"E2E Latency: {end_to_end_latency*1000:.2f}ms"
                                cv2.putText(annotated_rectifiedL, latency_text_e2e, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
                                send_status_to_firebase(robot_state, angles=angles_to_target[:3], ee_angle=config.INITIAL_EE_ANGLE)
                                last_object_detection_time = None
                            else:
                                send_status_to_firebase(robot_state, angles=angles_to_target[:3], ee_angle=config.INITIAL_EE_ANGLE)
                            move_start_time = time.time()
                    except ValueError as e:
                        send_status_to_firebase("ERROR", error=f"Error IK menuju objek: {e}")
                        robot_state = "IDLE"
            elif robot_state == "MOVING_TO_OBJECT":
                send_status_to_firebase(robot_state)
                if time.time() - move_start_time > config.MOVE_TO_OBJECT_DELAY:
                    send_status_to_firebase("GRABBING_OBJECT", angles=angles_to_target[:3], ee_angle=config.GRAB_EE_ANGLE)
                    if 'angles_to_target' in locals() and not np.isnan(angles_to_target).any():
                        send_serial_command(ser, "MOVE_ROBOT_ARM", angles_to_target[:3], config.GRAB_EE_ANGLE)
                        robot_state = "GRABBING"
                        send_status_to_firebase(robot_state, angles=angles_to_target[:3], ee_angle=config.GRAB_EE_ANGLE)
                        grab_start_time = time.time()
                    else:
                        send_status_to_firebase("ERROR", error="Sudut target tidak valid (NaN). Kembali ke IDLE.")
                        robot_state = "IDLE"
                        send_status_to_firebase(robot_state)
            elif robot_state == "GRABBING":
                send_status_to_firebase(robot_state)
                if time.time() - grab_start_time > config.GRAB_OBJECT_DELAY:
                    send_status_to_firebase("NO_PICKED", error="No <PICKED> feedback received")
                    send_serial_command(ser, "MOVE_ROBOT_ARM", config.INITIAL_ANGLES, config.GRAB_EE_ANGLE)
                    robot_state = "RETURNING"
                    send_status_to_firebase(robot_state, angles=config.INITIAL_ANGLES, ee_angle=config.GRAB_EE_ANGLE)
                    return_start_time = time.time()
            elif robot_state == "RETURNING":
                send_status_to_firebase(robot_state)
                if time.time() - return_start_time > config.RETURN_TO_INITIAL_DELAY:
                    send_serial_command(ser, "MOVE_ROBOT_ARM", config.INITIAL_ANGLES, config.INITIAL_EE_ANGLE)
                    robot_state = "OPENING"
                    send_status_to_firebase(robot_state, angles=config.INITIAL_ANGLES, ee_angle=config.INITIAL_EE_ANGLE)
                    open_start_time = time.time()
                    object_has_been_picked = False
            elif robot_state == "OPENING":
                send_status_to_firebase(robot_state)
                if time.time() - open_start_time > config.OPEN_EE_DELAY:
                    robot_state = "IDLE"
                    send_status_to_firebase(robot_state)
            elif robot_state == "_AFTER_PICK":
                send_status_to_firebase(robot_state)
                if time.time() - return_start_time > config.RETURN_TO_INITIAL_DELAY:
                    send_serial_command(ser, "MOVE_ROBOT_ARM", config.INITIAL_ANGLES, config.INITIAL_EE_ANGLE)
                    robot_state = "OPENING"
                    send_status_to_firebase(robot_state, angles=config.INITIAL_ANGLES, ee_angle=config.INITIAL_EE_ANGLE)
                    open_start_time = time.time()
                    object_has_been_picked = False

        key = cv2.waitKey(1)
        if key == 27:
            break

    if ser is not None:
        send_serial_command(ser, "MOVE_ROBOT_ARM", config.INITIAL_ANGLES, config.INITIAL_EE_ANGLE)
        ser.close()
    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()
    send_status_to_firebase("SHUTDOWN")
    send_chili_count_to_firebase(picked_chili_count)
    send_total_attempts_to_firebase(total_picking_attempts)