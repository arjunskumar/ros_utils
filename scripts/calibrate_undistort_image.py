import numpy as np
import cv2
import glob

chessboard_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
CHESSBOARD_SIZE = (6, 10)
calibration_flags = cv2.CALIB_RATIONAL_MODEL
term_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
obj_p = np.zeros((1, CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
obj_p[0, :, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

images = glob.glob(r'/home/arjun/Desktop/cam_calib_ws/results/img/*.png')

found = 0
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, chessboard_flags)
    if ret:
        objpoints.append(obj_p)
        corners2 = cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), subpix_criteria)
        imgpoints.append(corners2)
        img = cv2.drawChessboardCorners(img, CHESSBOARD_SIZE, corners2, ret)
        found += 1

print("Number of images used for calibration: ", found)

mtx = np.zeros((3, 3))
dist = np.zeros((8, 1))
rvec = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(found)]
tvec = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(found)]

# calibration
ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None, flags=calibration_flags,
                                           criteria=term_criteria)

print("K = np.array(" + str(mtx.tolist()) + ")")
print("D = np.array(" + str(dist.tolist()) + ")")
print("RMS = " + str(ret))


# Example undistortion
example_image_path = '/home/arjun/Desktop/cam_calib_ws/results/img1/frame000810.png'
example_image = cv2.imread(example_image_path)
h, w = example_image.shape[:2]
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

# Undistort the image
undistorted_image = cv2.undistort(example_image, mtx, dist, None, new_camera_matrix)

# Display the results
cv2.imshow('Original Image', example_image)
cv2.imshow('Undistorted Image', undistorted_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
