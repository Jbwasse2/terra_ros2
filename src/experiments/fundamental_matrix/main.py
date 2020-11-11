import os
import random

import matplotlib.pyplot as plt
import numpy as np
import pudb
from skimage import data, io
from skimage.color import rgb2gray
from skimage.feature import ORB, match_descriptors, plot_matches
from skimage.measure import ransac
from skimage.transform import FundamentalMatrixTransform
from tqdm import tqdm

from natsort import natsorted, ns

np.random.seed(0)

#Load images
image_list = []
list_files = os.listdir('./data/yt/')
list_files = natsorted(list_files)
for filename in list_files[60:70]:
    image_list.append(io.imread("./data/yt/" + filename))
    
matches_l = []
inliers_l = []
residual_l = []
avg_residual_l = []
descriptor_extractor = ORB()
random.shuffle(image_list)
for i in tqdm(range(len(image_list)-1)):
    img_left = image_list[0]
    img_right = image_list[i+1]
    img_left, img_right = map(rgb2gray, (img_left, img_right))
    # Find sparse feature correspondences between left and right image.
    descriptor_extractor.detect_and_extract(img_left)
    keypoints_left = descriptor_extractor.keypoints
    descriptors_left = descriptor_extractor.descriptors

    descriptor_extractor.detect_and_extract(img_right)
    keypoints_right = descriptor_extractor.keypoints
    descriptors_right = descriptor_extractor.descriptors

    matches = match_descriptors(descriptors_left, descriptors_right, cross_check=True)

    # Estimate the epipolar geometry between the left and right image.

    model, inliers = ransac(
        (keypoints_left[matches[:, 0]], keypoints_right[matches[:, 1]]),
        FundamentalMatrixTransform,
        min_samples=8,
        residual_threshold=10.0,
        max_trials=5000
    )

    inlier_keypoints_left = keypoints_left[matches[inliers, 0]]
    inlier_keypoints_right = keypoints_right[matches[inliers, 1]]

#    print(f"Number of matches: {matches.shape[0]}")
    matches_l.append(matches.shape[0])
#    print(f"Number of inliers: {inliers.sum()}")
    inliers_l.append(inliers.sum())
#    print("total residual")
    total_resid = np.sum(model.residuals(keypoints_left[matches[:, 0]], keypoints_right[matches[:, 1]]))
    residual_l.append(total_resid)
#    print(total_resid)
#    print("average residual")
    avg_resid = (1/len(keypoints_left))*np.sum(model.residuals(keypoints_left[matches[:, 0]], keypoints_right[matches[:, 1]]))
    avg_residual_l.append(avg_resid)

#    print(avg_resid)

    disp = inlier_keypoints_left[:, 1] - inlier_keypoints_right[:, 1]
    disp_coords = np.round(inlier_keypoints_left).astype(np.int64)
    disp_idxs = np.ravel_multi_index(disp_coords.T, img_left.shape)
    # disp_error = np.abs(groundtruth_disp.ravel()[disp_idxs] - disp)
    # disp_error = disp_error[np.isfinite(disp_error)]

    # Visualize the results.

#    fig, ax = plt.subplots(nrows=1, ncols=1)
#
#    plt.gray()
#
#    plot_matches(
#        ax,
#        img_left,
#        img_right,
#        keypoints_left,
#        keypoints_right,
#        matches[inliers],
#        only_matches=True,
#    )
#    ax.axis("off")
#    ax.set_title("Inlier correspondences")
#
#    # ax[1].hist(disp_error)
#    # ax[1].set_title("Histogram of disparity errors")
#
#    plt.show()
plt.plot(matches_l)
plt.savefig("matches.png")
plt.clf()
plt.plot(inliers_l)
plt.savefig("inliers.png")
plt.clf()
plt.plot(residual_l)
plt.savefig("residual.png")
plt.clf()
plt.plot(avg_residual_l)
plt.savefig("avgresidual.png")
plt.clf()
plt.plot([x/y for x,y in zip(inliers_l , matches_l)])
plt.savefig("percent_inlier.png")
plt.clf()
