import cv2
import numpy as np
import glob
import os

CHESSBOARD_SIZE = (5, 8)
SQUARE_SIZE = 0.03

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((1, CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[0,:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []
imgpoints_l = []
imgpoints_r = []

images_left = sorted(glob.glob('./captures3/left*.jpg'))
images_right = sorted(glob.glob('./captures3/right*.jpg'))

print("Left images:", images_left)
print("Right images:", images_right)

if len(images_left) == 0 or len(images_right) == 0:
    print("Error: No images found in 'captures' folder! Check path and naming.")
    exit()

if len(images_left) != len(images_right):
    print("Error: Number of left and right images must match!")
    exit()

for img_l_path, img_r_path in zip(images_left, images_right):
    left_idx = os.path.basename(img_l_path).replace('left', '').replace('.jpg', '')
    right_idx = os.path.basename(img_r_path).replace('right', '').replace('.jpg', '')
    if left_idx != right_idx:
        print(f"Error: Mismatched pair: {img_l_path} and {img_r_path}")
        exit()

print(f"Found {len(images_left)} image pairs")

print("\n--- Deteksi Sudut Papan Catur ---")
for img_l_path, img_r_path in zip(images_left, images_right):
    img_l = cv2.imread(img_l_path)
    img_r = cv2.imread(img_r_path)

    alpha = 2.0
    beta = 0
    img_l_processed = cv2.convertScaleAbs(img_l, alpha=alpha, beta=beta)
    img_r_processed = cv2.convertScaleAbs(img_r, alpha=alpha, beta=beta)

    gray_l = cv2.cvtColor(img_l_processed, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(img_r_processed, cv2.COLOR_BGR2GRAY)

    ret_l, corners_l = cv2.findChessboardCorners(gray_l, CHESSBOARD_SIZE, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    ret_r, corners_r = cv2.findChessboardCorners(gray_r, CHESSBOARD_SIZE, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    
    debug_folder = 'debug_corners'
    if not os.path.exists(debug_folder):
        os.makedirs(debug_folder)

    base_l = os.path.basename(img_l_path)
    base_r = os.path.basename(img_r_path)

    if ret_l == True:
        objpoints.append(objp)
        new_corners_l = cv2.cornerSubPix(gray_l, corners_l, (11, 11), (-1, -1), criteria)
        imgpoints_l.append(new_corners_l)
        img_l_drawn = cv2.drawChessboardCorners(img_l.copy(), CHESSBOARD_SIZE, new_corners_l, ret_l)
        cv2.imshow('Left Chessboard Detected', img_l_drawn)
        # cv2.imwrite(os.path.join(debug_folder, f'debug_{base_l}'), img_l_drawn)
    else:
        print(f"Chessboard not detected in left image: {img_l_path}")
        cv2.imshow('Left Chessboard Detected', img_l)
        # cv2.imwrite(os.path.join(debug_folder, f'debug_{base_l}'), img_l)

    if ret_r:
        new_corners_r = cv2.cornerSubPix(gray_r, corners_r, (11, 11), (-1, -1), criteria)
        imgpoints_r.append(new_corners_r)
        img_r_drawn = cv2.drawChessboardCorners(img_r.copy(), CHESSBOARD_SIZE, new_corners_r, ret_r)
        cv2.imshow('Right Chessboard Detected', img_r_drawn)
        # cv2.imwrite(os.path.join(debug_folder, f'debug_{base_r}'), img_r_drawn)
    else:
        print(f"Chessboard not detected in right image: {img_r_path}")
        cv2.imshow('Right Chessboard Detected', img_r)
        # cv2.imwrite(os.path.join(debug_folder, f'debug_{base_r}'), img_r)

    cv2.waitKey(500)
cv2.destroyAllWindows()


if len(objpoints) == 0:
    print("Error: No valid chessboard corners found in any image pair!")
    exit()

img_l = cv2.imread(images_left[0])
img_size = (img_l.shape[1], img_l.shape[0])

print("\n--- Proses Kalibrasi Kamera ---")
print("Calibrating left camera...")
ret_l, mtx_l, dist_l, rvecs_l, tvecs_l = cv2.calibrateCamera(
    objpoints, imgpoints_l, img_size, None, None)

print("Calibrating right camera...")
ret_r, mtx_r, dist_r, rvecs_r, tvecs_r = cv2.calibrateCamera(
    objpoints, imgpoints_r, img_size, None, None)

print("Performing stereo calibration...")
flags = cv2.CALIB_FIX_INTRINSIC
ret, mtx_l, dist_l, mtx_r, dist_r, R, T, E, F = cv2.stereoCalibrate(
    objpoints, imgpoints_l, imgpoints_r,
    mtx_l, dist_l, mtx_r, dist_r,
    img_size, criteria=criteria, flags=flags)

print("Computing stereo rectification...")
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
    mtx_l, dist_l, mtx_r, dist_r, img_size, R, T)

map1_l, map2_l = cv2.initUndistortRectifyMap(
    mtx_l, dist_l, R1, P1, img_size, cv2.CV_16SC2)
map1_r, map2_r = cv2.initUndistortRectifyMap(
    mtx_r, dist_r, R2, P2, img_size, cv2.CV_16SC2)

np.savez('stereo_calib.npz',
           mtx_l=mtx_l, dist_l=dist_l,
           mtx_r=mtx_r, dist_r=dist_r,
           R=R, T=T, E=E, F=F,
           R1=R1, R2=R2, P1=P1, P2=P2, Q=Q,
           map1_l=map1_l, map2_l=map2_l,
           map1_r=map1_r, map2_r=map2_r)

print("Calibration completed. Parameters saved to 'stereo_calib.npz'")

def rectify_images_and_display(img_l_path, img_r_path, output_path='rectified_display'):
    if not os.path.exists(output_path):
        os.makedirs(output_path)
    
    img_l = cv2.imread(img_l_path)
    img_r = cv2.imread(img_r_path)
    
    img_l_rect = cv2.remap(img_l, map1_l, map2_l, cv2.INTER_LINEAR)
    img_r_rect = cv2.remap(img_r, map1_r, map2_r, cv2.INTER_LINEAR)
    
    combined_rect = np.hstack((img_l_rect, img_r_rect))

    for i in range(0, combined_rect.shape[0], 20):
        cv2.line(combined_rect, (0, i), (combined_rect.shape[1], i), (0, 255, 0), 1)

    base_l = os.path.basename(img_l_path)
    base_r = os.path.basename(img_r_path)
    # cv2.imwrite(os.path.join(output_path, f'rect_{base_l}'), img_l_rect)
    # cv2.imwrite(os.path.join(output_path, f'rect_{base_r}'), img_r_rect)
    # cv2.imwrite(os.path.join(output_path, f'combined_rect_{base_l}'), combined_rect)

    cv2.imshow('Rectified Images with Epipolar Lines', combined_rect)



print("\n--- Menampilkan Gambar yang Direktifikasi ---")
for img_l_path, img_r_path in zip(images_left, images_right):
    rectify_images_and_display(img_l_path, img_r_path)

cv2.destroyAllWindows()
