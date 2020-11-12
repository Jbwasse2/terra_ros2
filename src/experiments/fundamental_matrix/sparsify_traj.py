import argparse
import os
import pickle

import numpy as np
import pudb
from matplotlib import pyplot as plt
from tqdm import tqdm

import cv2
from natsort import natsorted, ns

np.random.seed(0)


def calculate_residual(F, p1, p2):
    """
    Function to compute the residual average residual on frame 2
    param: F (3x3): fundamental matrix: (pt in frame 2).T * F * (pt in frame 1) = 0
    param: p1 (Nx2): 2d points on frame 1
    param: p2 (Nx2): 2d points on frame 2
    """
    P1 = np.c_[p1, np.ones((p1.shape[0], 1))].transpose()
    P2 = np.c_[p2, np.ones((p2.shape[0], 1))].transpose()
    L2 = np.matmul(F, P1).transpose()
    L2_norm = np.sqrt(L2[:, 0] ** 2 + L2[:, 1] ** 2)
    L2 = L2 / L2_norm[:, np.newaxis]
    pt_line_dist = np.multiply(L2, P2.T).sum(axis=1)
    return np.mean(np.square(pt_line_dist))


def get_residual(img1, img2, flann):
    import time

    MIN_MATCH_COUNT = 10
    # Initiate SIFT detector
    orb = cv2.ORB_create()

    pu.db
    # find the keypoints and descriptors with SIFT
    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)

    # FLANN parameters
    matches = flann.knnMatch(des1, des2, k=2)

    good = []
    pts1 = []
    pts2 = []

    #sometimes I get matches that are just a single or no points? This makes sure that matches are made
    matches = [x for x in matches if len(x) == 2]
    # ratio test as per Lowe's paper
    try:
        for i, (m, n) in enumerate(matches):
            if m.distance < 0.8 * n.distance:
                good.append(m)
                pts2.append(kp2[m.trainIdx].pt)
                pts1.append(kp1[m.queryIdx].pt)
    except Exception as e:
        print(e)
    pts1 = np.int32(pts1)
    pts2 = np.int32(pts2)
    start = time.time()
    F, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_LMEDS)

    # We select only inlier points
    pts1 = pts1[mask.ravel() == 1]
    pts2 = pts2[mask.ravel() == 1]
    res_in = calculate_residual(F, pts1, pts2)
    return res_in


def main(args):
    # Load images
    image_list = []
    residual_l = []
    list_files = os.listdir(args["data_location"])
    list_files = natsorted(list_files)
    look_aheads = []
    FLANN_INDEX_KDTREE = 0
#    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    FLANN_INDEX_LSH = 6
    index_params= dict(algorithm = FLANN_INDEX_LSH,
                       table_number = 12, # 12
                       key_size = 20,     # 20
                       multi_probe_level = 1) #2
    search_params = dict(checks=50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)
    for filename in list_files:
        image_list.append(cv2.imread(args["data_location"] + filename, 0))
    for i in tqdm(range(len(list_files) - 1 - int(args["look_ahead"]))):
        local_look_ahead = []
        for j in range(int(args["look_ahead"])):
            img1 = image_list[i]
            img2 = image_list[i + 1 + j]
            try:
                local_look_ahead.append(get_residual(img1, img2, flann))
            except Exception as e:
                print(e)
                local_look_ahead.append(np.inf)
        pu.db
        look_aheads.append(local_look_ahead)
    with open("residual.pkl", "wb") as f:
        pickle.dump(look_aheads, f)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Sparsifies a trajectory")
    parser.add_argument("-d", "--data_location", help="path to dir where traj data is")
    parser.add_argument("-l", "--look_ahead", help="length to look ahead")
    args = vars(parser.parse_args())
    main(args)
