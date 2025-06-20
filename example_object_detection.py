

from object_detection import detect_objects


if __name__ == '__main__':
    model_file = "models/si_mod_100_64_s.pt"
    detect_objects(model_path=model_file, source="camera")

    # image_to_detect = "images/1.png"
    # if os.path.exists(image_to_detect):
    #     print("\n--- Running Image Detection ---")
    #     detect_objects(model_path=model_file, source="image", image_path=image_to_detect)
    # else:
    #     print(f"\nSkipping image detection: Image file not found at {image_to_detect}")
