import numpy as np
import cv2
from ultralytics import YOLO
import os

import config

def load_stereo_calibration_data(calibration_file):
    """
    Memuat data kalibrasi stereo dari file .npz.
    """
    if not os.path.exists(calibration_file):
        print(f"Error: File kalibrasi tidak ditemukan di {calibration_file}. Harap kalibrasi kamera stereo Anda terlebih dahulu.")
        return None, None, None, None
    try:
        data = np.load(calibration_file)
        map1_l = data['map1_l']
        map2_l = data['map2_l']
        map1_r = data['map1_r']
        map2_r = data['map2_r']
        return map1_l, map2_l, map1_r, map2_r
    except KeyError as e:
        print(f"Error: Kunci kalibrasi tidak lengkap di {calibration_file}. Pastikan berisi map1_l, map2_l, map1_r, map2_r. Error: {e}")
        return None, None, None, None
    except Exception as e:
        print(f"Error memuat file kalibrasi {calibration_file}: {e}")
        return None, None, None, None

def detect_objects_stereo(model_path, calibration_file, camera_left_id=0, camera_right_id=1):
    model = YOLO(model_path)

    map1_l, map2_l, map1_r, map2_r = load_stereo_calibration_data(calibration_file)
    if map1_l is None:
        return # Keluar jika kalibrasi gagal dimuat

    cap_left = cv2.VideoCapture(camera_left_id)
    cap_right = cv2.VideoCapture(camera_right_id)

    if not (cap_left.isOpened() and cap_right.isOpened()):
        print(f"Error: Gagal membuka kamera. Pastikan ID kamera {camera_left_id} dan {camera_right_id} sudah benar.")
        if not cap_left.isOpened():
            print(f"Kamera kiri (ID: {camera_left_id}) gagal dibuka.")
        if not cap_right.isOpened():
            print(f"Kamera kanan (ID: {camera_right_id}) gagal dibuka.")
        return

    print("Memulai deteksi real-time dari kamera stereo dengan rectifikasi. Tekan 'q' untuk keluar.")

    while True:
        retL, frameL = cap_left.read()
        retR, frameR = cap_right.read()
        
        if not (retL and retR):
            print("Error: Gagal membaca frame dari salah satu atau kedua kamera. Keluar.")
            break

        # Rectifikasi kedua frame
        rectifiedL = cv2.remap(frameL, map1_l, map2_l, cv2.INTER_LINEAR)
        rectifiedR = cv2.remap(frameR, map1_r, map2_r, cv2.INTER_LINEAR)

        # Inferensi YOLO pada frame yang sudah direktifikasi
        conf = config.CONFIDENCE_THRESHOLD
        resultsL = model(rectifiedL, conf=conf, verbose=False)
        resultsR = model(rectifiedR, conf=conf, verbose=False)

        # Anotasi frame
        annotated_rectifiedL = resultsL[0].plot()
        annotated_rectifiedR = resultsR[0].plot()

        # Gabungkan frame untuk tampilan
        combined_annotated_frames = cv2.hconcat([annotated_rectifiedL, annotated_rectifiedR])
        
        # Opsional: ubah ukuran frame gabungan untuk tampilan
        # resized_combined_frames = cv2.resize(combined_annotated_frames, (1280, 480)) # Sesuaikan ukuran jika perlu
        
        cv2.imshow("Deteksi Objek Stereo (Kiri | Kanan Direkrifikasi)", combined_annotated_frames)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()
    print("Deteksi real-time dari kamera stereo dihentikan.")

if __name__ == "__main__":
    # Ganti dengan path model YOLO Anda
    YOLO_MODEL_PATH = config.MODEL_PATH
    # Ganti dengan path file kalibrasi stereo Anda
    STEREO_CALIBRATION_FILE = config.CALIBRATION_FILE

    # Jalankan deteksi untuk stereo camera
    detect_objects_stereo(YOLO_MODEL_PATH, STEREO_CALIBRATION_FILE, camera_left_id=0, camera_right_id=1)
    
    # Bagian kode untuk deteksi satu kamera atau gambar jika masih ingin dipertahankan
    # from ultralytics import YOLO
    # import cv2
    # import os 

    # def detect_objects(model_path, source="camera", image_path=None):
    #     model = YOLO(model_path)
    #     if source == "camera":
    #         cap = cv2.VideoCapture(0) 

    #         if not cap.isOpened():
    #             print("Error: Could not open webcam.")
    #             return

    #         print("Starting real-time detection from camera. Press 'q' to quit.")

    #         while True:
    #             ret, frame = cap.read()

    #             if not ret:
    #                 print("Error: Could not read frame from webcam. Exiting.")
    #                 break

    #             results = model(frame, conf=0.5, verbose=False)
    #             annotated_frame = results[0].plot()
    #             resized_frame = cv2.resize(annotated_frame, (800, 600))
    #             cv2.imshow("Real-Time Detected Objects (Camera)", resized_frame)

    #             if cv2.waitKey(1) & 0xFF == ord('q'):
    #                 break

    #         cap.release()
    #         cv2.destroyAllWindows()
    #         print("Real-time detection from camera stopped.")

    #     elif source == "image":
    #         if image_path is None:
    #             print("Error: image_path must be provided when source is 'image'.")
    #             return
    #         if not os.path.exists(image_path):
    #             print(f"Error: Image file not found at {image_path}")
    #             return

    #         print(f"Performing detection on image: {image_path}")
    #         image = cv2.imread(image_path)

    #         if image is None:
    #             print(f"Error: Could not read image at {image_path}. Check file format or corruption.")
    #             return

    #         results = model(image, conf=0.5, verbose=False)

    #         annotated_image = results[0].plot()
    #         resized_image = cv2.resize(annotated_image, (800, 600))
    #         cv2.imshow("Detected Objects (Image)", resized_image)
    #         print("Detection complete. Press any key to close the image.")
    #         cv2.waitKey(0)
    #         cv2.destroyAllWindows()
    #         print("Image detection closed.")

    #     else:
    #         print("Error: Invalid 'source' argument. Please use 'camera' or 'image'.")