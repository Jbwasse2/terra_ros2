import argparse
import os
import pickle

import matplotlib.pyplot as plt
import numpy as np
import pudb

import cv2
from natsort import natsorted, ns
from sparsify_traj import get_images_in_range, get_residual

parser = argparse.ArgumentParser(description="Sparsifies a trajectory")
parser.add_argument("-d", "--data_location", help="path to dir where traj data is")
args = vars(parser.parse_args())

with open('./foo.pkl', 'rb') as f:
    residuals = pickle.load(f)

list_files = os.listdir(args["data_location"])
list_files = natsorted(list_files)

for counter, l in enumerate(residuals):
    fig,axs = plt.subplots(1,2)
    l = np.array(l)
    pu.db
    img1 = get_images_in_range(counter,counter+1, list_files, args["data_location"])
    axs[0].imshow(cv2.cvtColor(img1, cv2.COLOR_BGR2RGB))
    a = np.argmin(l)
    img2 = get_images_in_range(a,a+1, list_files, args["data_location"])
    axs[1].imshow(cv2.cvtColor(img2, cv2.COLOR_BGR2RGB))
    plt.title(np.min(l))
    plt.show()
