import os
from ultralytics import YOLO
import cv2
import os 

def detect_objects(model_path, source="camera", image_path=None):
    model = YOLO(model_path)
    if source == "camera":
        cap = cv2.VideoCapture(0) 

        if not cap.isOpened():
            print("Error: Could not open webcam.")
            return

        print("Starting real-time detection from camera. Press 'q' to quit.")

        while True:
            ret, frame = cap.read()

            if not ret:
                print("Error: Could not read frame from webcam. Exiting.")
                break

            results = model(frame, conf=0.5, verbose=False)
            annotated_frame = results[0].plot()
            resized_frame = cv2.resize(annotated_frame, (800, 600))
            cv2.imshow("Real-Time Detected Objects (Camera)", resized_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        print("Real-time detection from camera stopped.")

    elif source == "image":
        if image_path is None:
            print("Error: image_path must be provided when source is 'image'.")
            return
        if not os.path.exists(image_path):
            print(f"Error: Image file not found at {image_path}")
            return

        print(f"Performing detection on image: {image_path}")
        image = cv2.imread(image_path)

        if image is None:
            print(f"Error: Could not read image at {image_path}. Check file format or corruption.")
            return

        results = model(image, conf=0.5, verbose=False)

        annotated_image = results[0].plot()
        resized_image = cv2.resize(annotated_image, (800, 600))
        cv2.imshow("Detected Objects (Image)", resized_image)
        print("Detection complete. Press any key to close the image.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        print("Image detection closed.")

    else:
        print("Error: Invalid 'source' argument. Please use 'camera' or 'image'.")




model_file = "models/si_100_60_s.pt"
detect_objects(model_path=model_file, source="camera")

# image_to_detect = "images/1.png"
# if os.path.exists(image_to_detect):
#     print("\n--- Running Image Detection ---")
#     detect_objects(model_path=model_file, source="image", image_path=image_to_detect)
# else:
#     print(f"\nSkipping image detection: Image file not found at {image_to_detect}")
