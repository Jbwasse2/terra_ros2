import argparse
import os

import numpy as np
import pudb
from matplotlib import pyplot as plt
from tqdm import tqdm

import cv2
from natsort import natsorted, ns

np.random.seed(0)

def fit_fundamental(matches):
    """
    Solves for the fundamental matrix using the matches with unnormalized method.
    """
    # <YOUR CODE>
    A = np.zeros((len(matches), 9))
    for counter, match in enumerate(matches):
        u = match[0]
        v = match[1]
        up = match[2]
        vp = match[3]
        t_vector = np.zeros((1, 9))
        t_vector[0, 0] = up * u
        t_vector[0, 1] = up * v
        t_vector[0, 2] = up
        t_vector[0, 3] = vp * u
        t_vector[0, 4] = vp * v
        t_vector[0, 5] = vp
        t_vector[0, 6] = u
        t_vector[0, 7] = v
        t_vector[0, 8] = 1
        A[counter, :] = t_vector
    U, S, V = np.linalg.svd(A)
    S[-1] = 0
    S = np.diag(S)
    m, n = A.shape
    U, S, V = np.linalg.svd(U[:, :n] @ S @ V[:m, :])
    x = V[-1, :]
    return x.reshape((3, 3))

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

    MIN_MATCH_COUNT = 10
    # Initiate SIFT detector
    orb = cv2.ORB_create()

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
            #if m.distance < 0.8 * n.distance:
            good.append(m)
            pts2.append(kp2[m.trainIdx].pt)
            pts1.append(kp1[m.queryIdx].pt)
    except Exception as e:
        print(e)
    pts1 = np.int32(pts1)
    pts2 = np.int32(pts2)
   # F1, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_LMEDS)
#    F, mask = cv2.findFundamentalMat(pts1, pts2, cv2.RANSAC)
    matches = np.hstack([pts1,pts2])
    #Average over res
    res_in2 = 0
    usedFlag = False
    for i in range(1000):
        if matches.shape[0] < 50:
            continue
        F2 = fit_fundamental(matches[np.random.choice(matches.shape[0], 8, replace=False)])
        temp = calculate_residual(F2, pts1, pts2)
        res_in2 += temp
        usedFlag = True
    res_in2 = res_in2 / 1000

    # We select only inlier points
#    pts1 = pts1[mask.ravel() == 1]
#    pts2 = pts2[mask.ravel() == 1]
#    res_in = calculate_residual(F1, pts1, pts2)
    return res_in2 if usedFlag else np.inf


def get_images_in_range(start, fin, list_files, data_location):
    image_list = []
    for i in range(start, fin):
        filename = list_files[i]
        img = cv2.imread(data_location + filename, 0)
        dim = (int(img.shape[1] * 0.25), int(img.shape[0] * 0.25))
        img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
        image_list.append(img)
    if len(image_list) == 1:
        return image_list[0]
    return image_list

def main(args):
    # Load images
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
    #This is used to skip over images that are close enough to other images in the trajectory
    skip_index = 0
    image_indeces = [0]
    for i in tqdm(range(len(list_files))):
        if i < skip_index:
            continue
        local_look_ahead = []
        img1 = get_images_in_range(i,i+1, list_files, args["data_location"])
        j = 0
        while(1):
            try:
                img2 = get_images_in_range(i+1+j, i + 2 + j, list_files, args["data_location"])
            except Exception as e:
                print(e)
                skip_index = i + j
                break
            j += 1
            try:
                residual = get_residual(img1, img2, flann)
                if residual > int(args["residual_constant"]):
                    skip_index = i + j
                    image_indeces.append(skip_index)
                    break
            except Exception as e:
                print(e)
    create_graphics(image_indeces, list_files, args)

def create_graphics(image_indeces, list_files, args):
    for counter, index in enumerate(image_indeces):
        filename = list_files[index]
        img = cv2.imread(args["data_location"] + filename)
        img_number = str(counter).zfill(4)
        cv2.imwrite(args["output_location"] + img_number + ".jpg", img)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Sparsifies a trajectory")
    parser.add_argument("-d", "--data_location", help="path to dir where traj data is")
    parser.add_argument("-o", "--output_location", help="where sparse trajectory gets saved to")
    parser.add_argument("-r", "--residual_constant", help="Used for sparsifying lower is less sparse, higher is more sparse (try 10)")
    args = vars(parser.parse_args())
    main(args)
