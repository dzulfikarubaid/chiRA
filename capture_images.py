import cv2
import os
import time

def capture_stereo_images(left_camera_index=0, right_camera_index=1, output_folder='captures'):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
        print(f"Created folder: {output_folder}")

    cap_left = cv2.VideoCapture(left_camera_index)
    cap_right = cv2.VideoCapture(right_camera_index)

    if not cap_left.isOpened():
        print(f"Error: Could not open left camera with index {left_camera_index}.")
        return
    if not cap_right.isOpened():
        print(f"Error: Could not open right camera with index {right_camera_index}.")
        return

    print("Press SPACE to capture images.")
    print("Press 'q' to quit.")

    image_count = 0
    while True:
        ret_left, frame_left = cap_left.read()
        ret_right, frame_right = cap_right.read()

        if not ret_left or not ret_right:
            print("Error: Failed to grab frames from one or both cameras.")
            break

        cv2.imshow('Left Camera (Press SPACE to capture)', frame_left)
        cv2.imshow('Right Camera (Press SPACE to capture)', frame_right)

        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):
            timestamp = int(time.time())
            left_filename = os.path.join(output_folder, f'left_{image_count:04d}.jpg')
            right_filename = os.path.join(output_folder, f'right_{image_count:04d}.jpg')

            cv2.imwrite(left_filename, frame_left)
            cv2.imwrite(right_filename, frame_right)
            print(f"Captured image pair {image_count}: {left_filename}, {right_filename}")
            image_count += 1
            time.sleep(0.2)

        elif key == ord('q'):
            print("Exiting image capture.")
            break

    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_stereo_images(left_camera_index=0, right_camera_index=1, output_folder='./captures3')