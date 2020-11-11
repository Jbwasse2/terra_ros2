import os

import numpy as np
import pudb
from matplotlib import pyplot as plt
from tqdm import tqdm

import cv2
from natsort import natsorted, ns


def get_residual(F, p1, p2):
    """
    Function to compute the residual average residual on frame 2
    param: F (3x3): fundamental matrix: (pt in frame 2).T * F * (pt in frame 1) = 0
    param: p1 (Nx2): 2d points on frame 1
    param: p2 (Nx2): 2d points on frame 2
    """
    P1 = np.c_[p1, np.ones((p1.shape[0],1))].transpose()
    P2 = np.c_[p2, np.ones((p2.shape[0],1))].transpose()
    L2 = np.matmul(F, P1).transpose()
    L2_norm = np.sqrt(L2[:,0]**2 + L2[:,1]**2)
    L2 = L2 / L2_norm[:,np.newaxis]
    pt_line_dist = np.multiply(L2, P2.T).sum(axis = 1)
    return np.mean(np.square(pt_line_dist))

np.random.seed(0)

#Load images
image_list = []
residual_l = []
list_files = os.listdir('./data/yt/')
list_files = natsorted(list_files)
for filename in list_files[35:50]:
    image_list.append(cv2.imread("./data/yt/" + filename, 0))

MIN_MATCH_COUNT = 10
res_l = []
res_in_l = []

for i in tqdm(range(len(image_list))):
    img1 = image_list[0]
    img2 = image_list[i]

    # Initiate SIFT detector
    sift = cv2.SIFT_create()

    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)


    # FLANN parameters
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=50)

    flann = cv2.FlannBasedMatcher(index_params,search_params)
    matches = flann.knnMatch(des1,des2,k=2)

    good = []
    pts1 = []
    pts2 = []

    # ratio test as per Lowe's paper
    for i,(m,n) in enumerate(matches):
        if m.distance < 0.8*n.distance:
            good.append(m)
            pts2.append(kp2[m.trainIdx].pt)
            pts1.append(kp1[m.queryIdx].pt)
    pts1 = np.int32(pts1)
    pts2 = np.int32(pts2)
    F, mask = cv2.findFundamentalMat(pts1,pts2,cv2.FM_LMEDS)
    res = get_residual(F, pts1, pts2)
    res_l.append(res)

    # We select only inlier points
    pts1 = pts1[mask.ravel()==1]
    pts2 = pts2[mask.ravel()==1]
    res_in = get_residual(F, pts1, pts2)
    res_in_l.append(res_in)
plt.plot(res_in_l)
plt.savefig("res_in.png")
plt.clf()
plt.plot(res_l)
plt.savefig("res_all.png")
plt.clf()
