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
cred = credentials.Certificate("key.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://ecs-oprec-default-rtdb.asia-southeast1.firebasedatabase.app/'
})
firebase_ref = db.reference('robot_status')

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
            'x': detected_object_position_cm[0],
            'y': detected_object_position_cm[1],
            'z': detected_object_position_cm[2]
        }
    if error:
        status_data['error'] = error
    if picked:
        status_data['picked'] = picked
    try:
        firebase_ref.set(status_data)
        print(f"Status '{status}' dikirim ke Firebase.")
    except Exception as e:
        print(f"Gagal mengirim status ke Firebase: {e}")

SERIAL_PORT = '/dev/cu.usbmodem146101'
BAUD_RATE = 115200
TIME_OUT = 2

MODEL_PATH = 'models/chili11n_64_100_1_openvino_model'
CONFIDENCE_THRESHOLD = 0.7
FILTER = True

L1 = 9.5
L2 = 13.5
L3 = 14.7
L4 = 10

INITIAL_ANGLES = (0.0, 80.0, 160.0)
INITIAL_EE_ANGLE = 90.0

GRAB_EE_ANGLE = 0.0


def clamp_angle(angle, min_angle, max_angle):
    return max(min(angle, max_angle), min_angle)

def send_serial_command(ser, command_type, angles, ee_angle, move_time=1000):
    theta1, theta2, theta3 = angles
    data_to_send = f"<{command_type},{ee_angle:.2f},{theta1:.2f},{theta2:.2f},{theta3:.2f},{move_time},{move_time},{move_time},{move_time}>"
    print(f"Mengirim: {data_to_send}")
    if ser:
        ser.write(data_to_send.encode())
    send_status_to_firebase("SENDING_COMMAND", angles=(theta1, theta2, theta3), ee_angle=ee_angle)
    time.sleep(0.1)

def calculate_distance(point_left, point_right, Q):
    disparity = point_left[0] - point_right[0]
    if disparity <= 0:
        return None
    pw = np.array([point_left[0], point_left[1], disparity, 1], dtype=np.float32).reshape(-1, 1)
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
def find_matching_objects(resultsL, resultsR, threshold_y=10, threshold_class=0.8):
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
                if dy < threshold_y and class_id_L == class_id_R:
                    if dy < min_dy:
                        min_dy = dy
                        best_match_idx = j
        if best_match_idx != -1:
            matches.append((i, best_match_idx))
            used_right[best_match_idx] = True
    return matches, boxesL, class_idsL, boxesR, class_idsR

if __name__ == "__main__":
    try:
        model = YOLO(MODEL_PATH, task="detect")
        calibration_data = np.load('camera_calibration/stereo_calib.npz')
        if not all(k in calibration_data for k in ['map1_l', 'map2_l', 'map1_r', 'map2_r', 'Q']):
            raise KeyError("Kunci kalibrasi tidak lengkap (pastikan ada 'Q')")
        Q = calibration_data['Q']
    except Exception as e:
        print(f"Error memuat model atau kalibrasi: {e}")
        send_status_to_firebase("ERROR", error=str(e))
        exit()

    cap_left = cv2.VideoCapture(1)
    cap_right = cv2.VideoCapture(0)

    if not (cap_left.isOpened() and cap_right.isOpened()):
        print("Gagal membuka salah satu kamera")
        send_status_to_firebase("ERROR", error="Gagal membuka salah satu kamera")
        exit()

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIME_OUT)
        print(f"Terhubung ke serial port {SERIAL_PORT}")
        time.sleep(2)
    except serial.SerialException as e:
        print(f"Gagal membuka port serial {SERIAL_PORT}: {e}")
        send_status_to_firebase("ERROR", error=f"Gagal membuka port serial {SERIAL_PORT}: {e}")
        ser = None

    robot_state = "IDLE"
    send_status_to_firebase(robot_state, INITIAL_ANGLES, INITIAL_EE_ANGLE)
    object_has_been_picked = False

    while True:
        retL, frameL = cap_left.read()
        retR, frameR = cap_right.read()
        
        if not (retL and retR):
            print("Gagal membaca frame dari kamera")
            send_status_to_firebase("ERROR", error="Gagal membaca frame dari kamera")
            continue
        
        if(FILTER):
            frameL = adjust_saturation_brightness(frameL)
            frameR = adjust_saturation_brightness(frameR)

        rectifiedL = cv2.remap(frameL, calibration_data['map1_l'], calibration_data['map2_l'], cv2.INTER_LINEAR)
        rectifiedR = cv2.remap(frameR, calibration_data['map1_r'], calibration_data['map2_r'], cv2.INTER_LINEAR)

        resultsL = model(rectifiedL, verbose=False, conf=CONFIDENCE_THRESHOLD)
        resultsR = model(rectifiedR, verbose=False, conf=CONFIDENCE_THRESHOLD)

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

            cv2.rectangle(annotated_rectifiedL, (x1L, y1L), (x2L, y2L), (0, 255, 0), 2)
            cv2.rectangle(annotated_rectifiedR, (x1R, y1R), (x2R, y2R), (0, 255, 0), 2)
            cv2.circle(annotated_rectifiedL, center_L, 5, (0, 100, 255), -1)

            if distance_m is not None:
                X_m, Y_m, Z_m = distance_m
                detected_object_position_m = (X_m, Y_m, Z_m)
                h = np.sin(60*np.pi/180) * L2
                y_adjustment = 30
                z_adjustment = 24-15
                robot_z_cm = Z_m * 100 - z_adjustment
                robot_x_cm = -X_m * 100
                robot_y_cm = -Y_m * 100 + y_adjustment
                detected_object_position_cm = (robot_x_cm, robot_y_cm, robot_z_cm)
                distance_text = f"X:{robot_x_cm:.2f} Y:{robot_y_cm:.2f} Z:{robot_z_cm:.2f}cm"
                print(distance_text)
                cv2.putText(annotated_rectifiedL, distance_text, (x1L, y1L - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                break

        try:
            combined_rectified = cv2.hconcat([annotated_rectifiedL, annotated_rectifiedR])
            cv2.imshow("Stereo Rectified with Detection (Left | Right)", combined_rectified)
        except Exception as e:
            print(f"Error menampilkan frame: {e}")

        if ser is not None:
            if ser.in_waiting > 0:
                feedback = ser.readline().decode().strip()
                print(f"Feedback from Arduino: {feedback}")
                if feedback == "<PICKED>":
                    print("Objek telah dipetik!")
                    object_has_been_picked = True
                    send_status_to_firebase("PICKED", picked=True)

            if robot_state == "IDLE" and not object_has_been_picked:
                send_status_to_firebase(robot_state)
                if detected_object_position_cm:
                    obj_x_cm, obj_y_cm, obj_z_cm = detected_object_position_cm
                    print(f"Posisi objek (cm): X={obj_x_cm:.2f}, Y={obj_y_cm:.2f}, Z={obj_z_cm:.2f}")
                    send_status_to_firebase("OBJECT_DETECTED", detected_object_position_cm=detected_object_position_cm)
                    print("Objek terdeteksi. Bergerak menuju objek (EE terbuka)...")
                    try:
                        angles_to_target = calculate_angles(obj_x_cm, obj_y_cm, obj_z_cm)
                        print(f"Sudut target: {angles_to_target}")
                        if not np.isnan(angles_to_target).any():
                            send_serial_command(ser, "HelloWorld", angles_to_target[:3], INITIAL_EE_ANGLE)
                            robot_state = "MOVING_TO_OBJECT"
                            send_status_to_firebase(robot_state, angles=angles_to_target[:3], ee_angle=INITIAL_EE_ANGLE)
                            move_start_time = time.time()
                        else:
                            print("Error: Perhitungan sudut menghasilkan NaN. Tidak mengirim perintah serial.")
                            send_status_to_firebase("ERROR", error="Perhitungan sudut menghasilkan NaN.")
                            robot_state = "IDLE"
                    except ValueError as e:
                        print(f"Error IK menuju objek: {e}")
                        send_status_to_firebase("ERROR", error=f"Error IK menuju objek: {e}")
                        robot_state = "IDLE"
                else:
                    print("Mencari objek...")
            elif robot_state == "MOVING_TO_OBJECT" and not object_has_been_picked:
                send_status_to_firebase(robot_state)
                if time.time() - move_start_time > 3:
                    print("Mencengkram objek...")
                    send_status_to_firebase("GRABBING_OBJECT", angles=angles_to_target[:3], ee_angle=GRAB_EE_ANGLE)
                    if 'angles_to_target' in locals() and not np.isnan(angles_to_target).any():
                        send_serial_command(ser, "HelloWorld", angles_to_target[:3], GRAB_EE_ANGLE)
                        robot_state = "GRABBING"
                        send_status_to_firebase(robot_state, angles=angles_to_target[:3], ee_angle=GRAB_EE_ANGLE)
                        grab_start_time = time.time()
                    else:
                        print("Error: Sudut target tidak valid (NaN). Tidak dapat mencengkram.")
                        send_status_to_firebase("ERROR", error="Sudut target tidak valid (NaN). Kembali ke IDLE.")
                        robot_state = "IDLE"
                        send_status_to_firebase(robot_state)
            elif robot_state == "GRABBING" and not object_has_been_picked:
                send_status_to_firebase(robot_state)
                if time.time() - grab_start_time > 1:
                    print("Kembali ke posisi awal (EE tertutup)...")
                    send_serial_command(ser, "HelloWorld", INITIAL_ANGLES, GRAB_EE_ANGLE)
                    robot_state = "RETURNING"
                    send_status_to_firebase(robot_state, angles=INITIAL_ANGLES, ee_angle=GRAB_EE_ANGLE)
                    return_start_time = time.time()
            elif robot_state == "RETURNING" and not object_has_been_picked:
                send_status_to_firebase(robot_state)
                if time.time() - return_start_time > 1:
                    print("Membuka end effector di posisi awal...")
                    send_serial_command(ser, "HelloWorld", INITIAL_ANGLES, INITIAL_EE_ANGLE)
                    robot_state = "OPENING"
                    send_status_to_firebase(robot_state, angles=INITIAL_ANGLES, ee_angle=INITIAL_EE_ANGLE)
                    open_start_time = time.time()
            elif robot_state == "OPENING" and not object_has_been_picked:
                send_status_to_firebase(robot_state)
                if time.time() - open_start_time > 1:
                    robot_state = "IDLE"
                    send_status_to_firebase(robot_state)
            elif object_has_been_picked:
                print("Objek sudah dipetik. Menunggu perintah selanjutnya atau reset.")

        key = cv2.waitKey(1)
        if key == 27:
            break

    if ser is not None:
        send_serial_command(ser, "HelloWorld", INITIAL_ANGLES, INITIAL_EE_ANGLE)
        ser.close()
    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()
    send_status_to_firebase("SHUTDOWN")