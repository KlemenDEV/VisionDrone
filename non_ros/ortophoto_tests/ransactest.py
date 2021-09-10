import cv2
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    img1 = cv2.imread("./test_samples/direct/2a.png", cv2.IMREAD_GRAYSCALE)
    img2 = cv2.imread("./test_samples/direct/1a.png", cv2.IMREAD_GRAYSCALE)

    # Undistort dron image
    mtx = np.array([
        [1276.704618338571, 0, 634.8876509199106],
        [0, 1274.342831275509, 379.8318028940378],
        [0, 0, 1],
    ])
    dist = np.array([0.1465167016954302, -0.2847343180128725, 0.00134017721235817, -0.004309553450829512, 0])
    h, w = img2.shape
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    img2 = cv2.undistort(img2, mtx, dist, None, newcameramtx)

    # ORB
    orb = cv2.ORB_create()
    keypoints_1, descriptors_1 = orb.detectAndCompute(img1, None)
    keypoints_2, descriptors_2 = orb.detectAndCompute(img2, None)

    # Feature matching
    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
    matches = bf.match(descriptors_1, descriptors_2)

    img3 = cv2.drawMatches(img1, keypoints_1, img2, keypoints_2, matches, img2, flags=2)

    # Homography using ransac
    src_pts = np.float32([keypoints_1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([keypoints_2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
    M, status = cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 0.0005)

    # Draw transform
    h, w = img2.shape
    pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
    dst = cv2.perspectiveTransform(pts, M)
    img3 = cv2.polylines(img3, [np.int32(dst)], True, (0, 255, 0), 2, cv2.LINE_AA)

    plt.figure(figsize=(10, 10))
    plt.imshow(img3)
    plt.show()
