from collections import Counter

import cv2
import matplotlib.pyplot as plt
import numpy
import numpy as np
import rasterio
from pyproj import Proj
from sklearn.cluster import AgglomerativeClustering

if __name__ == "__main__":
    fname = './map/F053862.tif'

    # Read raster
    with rasterio.open(fname) as r:
        T0 = r.transform  # upper-left pixel corner affine transform
        p1 = Proj(r.crs)
        A = r.read()  # pixel values

    img = np.transpose(np.delete(A, 3, axis=0).T, (1, 0, 2))

    # plt.imshow(img)
    # plt.show()

    # https://gis.stackexchange.com/questions/129847/obtain-coordinates-and-corresponding-pixel-values-from-geotiff-using-python-gdal

    # All rows and columns
    # cols, rows = np.meshgrid(np.arange(A.shape[2]), np.arange(A.shape[1]))

    # Get affine transform for pixel centres
    # T1 = T0 * Affine.translation(0.5, 0.5)

    # Function to convert pixel row/column index (from 0) to easting/northing at centre
    # rc2en = lambda r, c: (c, r) * T1

    # All eastings and northings (there is probably a faster way to do this)
    # eastings, northings = np.vectorize(rc2en, otypes=[float, float])(rows, cols)

    # Project all longitudes, latitudes
    # p2 = Proj(proj='latlong', datum='WGS84')
    # longs, lats = transform(p1, p2, eastings, northings)

    # https://towardsdatascience.com/improving-your-image-matching-results-by-14-with-one-line-of-code-b72ae9ca2b73
    # https://docs.opencv.org/master/dc/dc3/tutorial_py_matcher.html
    # https://www.analyticsvidhya.com/blog/2019/10/detailed-guide-powerful-sift-technique-image-matching-python/

    img1 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img2 = cv2.imread("./test_samples/frame000095.png", cv2.IMREAD_GRAYSCALE)

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
    # x, y, w, h = roi
    # img2 = img2[y:y + h, x:x + w]
    # -- undistort end

    # Initiate ORB detector
    orb = cv2.ORB_create(2000)

    keypoints_1, descriptors_1 = orb.detectAndCompute(img1, None)
    keypoints_2, descriptors_2 = orb.detectAndCompute(img2, None)

    # feature matching
    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)

    matches = bf.match(descriptors_1, descriptors_2)

    points = [keypoints_1[m.queryIdx].pt for m in matches]

    cluster = AgglomerativeClustering(n_clusters=None, affinity='euclidean', linkage='complete', distance_threshold=500)
    labels = cluster.fit_predict(points)
    ldata = Counter(labels)
    largest_cluster = max(set(labels), key=ldata.get)

    x, y, cnt = 0, 0, 0
    for idx, p in enumerate(points):
        if labels[idx] == largest_cluster:
            x += p[0]
            y += p[1]
            cnt += 1

    cluster_center = (int(x / cnt), int(y / cnt))

    gmatches = []
    for m in matches:
        # if numpy.linalg.norm(np.array(keypoints_1[m.queryIdx].pt) - np.array(cluster_center)) < 750:
        gmatches.append(m)
    # gmatches = sorted(gmatches, key=lambda x: x.distance)

    # img3 = cv2.drawMatches(img1, keypoints_1, img2, keypoints_2, matches, img2, flags=2)
    img3 = cv2.drawMatches(img1, keypoints_1, img2, keypoints_2, gmatches, img2, flags=2)

    for m in gmatches:
        img3 = cv2.circle(img3, (int(keypoints_1[m.queryIdx].pt[0]), int(keypoints_1[m.queryIdx].pt[1])), 10,
                          (255, 255, 0), 1)
    img3 = cv2.circle(img3, cluster_center, 25, (0, 255, 255), 10)

    # https://docs.opencv.org/3.4/d1/de0/tutorial_py_feature_homography.html
    src_pts = np.float32([keypoints_1[m.queryIdx].pt for m in gmatches]).reshape(-1, 1, 2)
    dst_pts = np.float32([keypoints_2[m.trainIdx].pt for m in gmatches]).reshape(-1, 1, 2)
    M, status = cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 0.0005)

    h, w = img2.shape
    pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
    dst = cv2.perspectiveTransform(pts, M)

    # img3 = cv2.polylines(img3, [np.int32(dst)], True, (255, 0, 0), 15, cv2.LINE_AA)

    plt.figure(figsize=(10, 10))
    plt.imshow(img3)
    plt.show()
