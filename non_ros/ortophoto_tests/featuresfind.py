import cv2
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":

    img1 = cv2.imread("./test_samples/direct/1b.png", cv2.IMREAD_GRAYSCALE)
    img2 = cv2.imread("./test_samples/direct/2b.png", cv2.IMREAD_GRAYSCALE)

    # -- undistort start
    # img2 = cv2.resize(img2, (1280, 720), interpolation=cv2.INTER_AREA)
    mtx = np.array([
        [1276.704618338571, 0, 634.8876509199106],
        [0, 1274.342831275509, 379.8318028940378],
        [0, 0, 1],
    ])
    dist = np.array([0.1465167016954302, -0.2847343180128725, 0.00134017721235817, -0.004309553450829512, 0])
    h, w = img2.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    img2 = cv2.undistort(img2, mtx, dist, None, newcameramtx)
    # -- undistort end

    # Initiate ORB detector
    orb = cv2.ORB_create()

    keypoints_1, descriptors_1 = orb.detectAndCompute(img1, None)
    keypoints_2, descriptors_2 = orb.detectAndCompute(img2, None)

    # feature matching
    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)

    matches = bf.match(descriptors_1, descriptors_2)

    points = [keypoints_1[m.queryIdx].pt for m in matches]

    img3 = cv2.drawMatches(img1, keypoints_1, img2, keypoints_2, matches, img2, flags=2)

    for m in matches:
        img3 = cv2.circle(img3, (int(keypoints_1[m.queryIdx].pt[0]), int(keypoints_1[m.queryIdx].pt[1])), 10,
                          (255, 255, 0), 1)

    img1 = cv2.cvtColor(img1, cv2.COLOR_GRAY2RGB)
    img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2RGB)

    for k in keypoints_1:
        img1 = cv2.circle(img1, (int(k.pt[0]), int(k.pt[1])), 3, (255, 255, 0), 1)

    for k in keypoints_2:
        img2 = cv2.circle(img2, (int(k.pt[0]), int(k.pt[1])), 3, (255, 255, 0), 1)

    plt.figure(figsize=(10, 10))
    plt.imshow(img3)

    plt.figure(figsize=(10, 10))
    plt.imshow(img1)

    plt.figure(figsize=(10, 10))
    plt.imshow(img2)
    plt.show()

    cv2.waitKey()
