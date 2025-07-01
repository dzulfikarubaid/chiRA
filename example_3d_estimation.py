import numpy as np
import cv2
from ultralytics import YOLO
import os

# Diasumsikan Anda memiliki file config.py seperti ini:
# config.py
# MODEL_PATH = "path/to/your/yolov8n.pt"
# CALIBRATION_FILE = "path/to/your/stereo_calibration.npz"
import config

def load_stereo_calibration_data(calibration_file):
    """
    Memuat data kalibrasi stereo dari file .npz.
    Sekarang juga memuat matriks Q yang penting untuk perhitungan 3D.
    """
    if not os.path.exists(calibration_file):
        print(f"Error: File kalibrasi tidak ditemukan di {calibration_file}. Harap kalibrasi kamera stereo Anda terlebih dahulu.")
        return None, None, None, None, None
    try:
        data = np.load(calibration_file)
        # Memastikan semua kunci yang diperlukan ada
        required_keys = ['map1_l', 'map2_l', 'map1_r', 'map2_r', 'Q']
        for key in required_keys:
            if key not in data:
                raise KeyError(f"Kunci kalibrasi '{key}' tidak ditemukan di file.")

        map1_l = data['map1_l']
        map2_l = data['map2_l']
        map1_r = data['map1_r']
        map2_r = data['map2_r']
        Q = data['Q']
        return map1_l, map2_l, map1_r, map2_r, Q
    except KeyError as e:
        print(f"Error: Kunci kalibrasi tidak lengkap di {calibration_file}. {e}")
        return None, None, None, None, None
    except Exception as e:
        print(f"Error memuat file kalibrasi {calibration_file}: {e}")
        return None, None, None, None, None

def match_detected_objects(resultsL, resultsR, y_tolerance=30):
    """
    Mencocokkan objek yang terdeteksi di frame kiri dan kanan.
    Kecocokan didasarkan pada kelas yang sama dan posisi vertikal (y) yang berdekatan.
    """
    matches = []
    boxesL = resultsL[0].boxes.data.cpu().numpy()
    boxesR = resultsR[0].boxes.data.cpu().numpy()
    
    for boxL in boxesL:
        x1_l, y1_l, x2_l, y2_l, conf_l, class_id_l = boxL
        center_y_l = (y1_l + y2_l) / 2
        
        best_match = None
        min_y_diff = float('inf')

        for boxR in boxesR:
            x1_r, y1_r, x2_r, y2_r, conf_r, class_id_r = boxR
            
            # Periksa apakah kelasnya sama
            if class_id_l == class_id_r:
                center_y_r = (y1_r + y2_r) / 2
                y_diff = abs(center_y_l - center_y_r)
                
                # Periksa apakah berada dalam toleransi vertikal dan merupakan yang terbaik sejauh ini
                if y_diff < y_tolerance and y_diff < min_y_diff:
                    min_y_diff = y_diff
                    best_match = boxR
        
        if best_match is not None:
            matches.append((boxL, best_match))
            
    return matches

def estimate_3d_coordinates(model_path, calibration_file, camera_left_id=0, camera_right_id=1):
    """
    Fungsi utama untuk mendeteksi objek dan mengestimasi koordinat 3D mereka secara real-time.
    """
    model = YOLO(model_path)

    map1_l, map2_l, map1_r, map2_r, Q = load_stereo_calibration_data(calibration_file)
    if Q is None:
        print("Gagal memuat matriks Q. Estimasi 3D tidak dapat dilanjutkan.")
        return

    cap_left = cv2.VideoCapture(camera_left_id)
    cap_right = cv2.VideoCapture(camera_right_id)

    if not (cap_left.isOpened() and cap_right.isOpened()):
        print(f"Error: Gagal membuka kamera. Pastikan ID kamera {camera_left_id} dan {camera_right_id} sudah benar.")
        return

    print("Memulai deteksi real-time dan estimasi 3D. Tekan 'q' untuk keluar.")

    while True:
        retL, frameL = cap_left.read()
        retR, frameR = cap_right.read()
        
        if not (retL and retR):
            print("Error: Gagal membaca frame. Keluar.")
            break

        # 1. Rektifikasi kedua frame
        rectifiedL = cv2.remap(frameL, map1_l, map2_l, cv2.INTER_LINEAR)
        rectifiedR = cv2.remap(frameR, map1_r, map2_r, cv2.INTER_LINEAR)

        # 2. Inferensi YOLO pada frame yang sudah direktifikasi
        resultsL = model(rectifiedL, conf=0.5, verbose=False)
        resultsR = model(rectifiedR, conf=0.5, verbose=False)

        # 3. Anotasi frame (dilakukan setelah perhitungan agar tidak mengganggu)
        annotated_rectifiedL = resultsL[0].plot()
        annotated_rectifiedR = resultsR[0].plot()

        # 4. Mencocokkan objek antara frame kiri dan kanan
        matched_objects = match_detected_objects(resultsL, resultsR)

        # 5. Hitung koordinat 3D untuk setiap pasangan yang cocok
        for boxL, boxR in matched_objects:
            # Dapatkan koordinat pusat untuk kedua kotak
            x1_l, y1_l, x2_l, y2_l, _, _ = boxL
            center_x_l = int((x1_l + x2_l) / 2)
            center_y_l = int((y1_l + y2_l) / 2)
            
            x1_r, y1_r, x2_r, y2_r, _, _ = boxR
            center_x_r = int((x1_r + x2_r) / 2)

            # Hitung disparitas (perbedaan x)
            disparity = abs(center_x_l - center_x_r)

            if disparity > 0:
                # Buat vektor 4D (koordinat gambar dan disparitas)
                point_4d_hom = np.array([center_x_l, center_y_l, disparity, 1.0])
                
                # Lakukan perkalian matriks dengan Q
                point_3d_hom = Q @ point_4d_hom
                
                # Lakukan de-homogenisasi (bagi dengan w)
                w = point_3d_hom[3]
                
                if w != 0:
                    x = point_3d_hom[0] / w
                    y = point_3d_hom[1] / w
                    z = point_3d_hom[2] / w
                    
    
                    x_cm = x / 10.0
                    y_cm = y / 10.0
                    z_cm = z / 10.0
                    
                    coord_text = f"X:{x_cm:.1f} Y:{y_cm:.1f} Z:{z_cm:.1f} cm"
                    print(coord_text)
                    cv2.putText(annotated_rectifiedL, coord_text, (int(x1_l), int(y1_l) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Gabungkan frame untuk tampilan
        combined_annotated_frames = cv2.hconcat([annotated_rectifiedL, annotated_rectifiedR])
        
        cv2.imshow("Estimasi 3D Stereo (Kiri | Kanan Direkrifikasi)", combined_annotated_frames)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()
    print("Deteksi dan estimasi dihentikan.")
if __name__ == "__main__":
    YOLO_MODEL_PATH = config.MODEL_PATH
    STEREO_CALIBRATION_FILE = config.CALIBRATION_FILE
    
    # Ganti ID kamera jika perlu, misalnya menggunakan file video:
    # LEFT_SOURCE = "path/to/left_video.mp4"
    # RIGHT_SOURCE = "path/to/right_video.mp4"
    # estimate_3d_coordinates(YOLO_MODEL_PATH, STEREO_CALIBRATION_FILE, camera_left_id=LEFT_SOURCE, camera_right_id=RIGHT_SOURCE)
    
    estimate_3d_coordinates(YOLO_MODEL_PATH, STEREO_CALIBRATION_FILE, camera_left_id=0, camera_right_id=1)