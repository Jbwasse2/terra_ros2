import argparse
import os
import pickle
import re

import numpy as np
import pudb
from matplotlib import pyplot as plt
from tqdm import tqdm

import cv2
from natsort import natsorted, ns
from sparsify_traj import get_images_in_range, get_residual

#(ux, uy, vx, vy)

parser = argparse.ArgumentParser(description="Sparsifies a trajectory")
parser.add_argument("-d", "--data_location", help="path to dir where traj data is")
parser.add_argument("-r", "--residual_constant", help="Used for sparsifying lower is less sparse, higher is more sparse (try 10)")
args = vars(parser.parse_args())
list_files = os.listdir(args["data_location"])
regexp = re.compile(r'.*\_2.*')
list_files_2 = list(filter(regexp.match, list_files))
list_files = list( set(list_files) - set(list_files_2))
list_files = natsorted(list_files)
list_files_2 = natsorted(list_files_2)
FLANN_INDEX_LSH = 6
index_params= dict(algorithm = FLANN_INDEX_LSH,
                   table_number = 12, # 12
                   key_size = 20,     # 20
                   multi_probe_level = 1) #2
search_params = dict(checks=50)

flann = cv2.FlannBasedMatcher(index_params, search_params)

def show_img(i, l, list_files, args):
    fig,axs = plt.subplots(1,2)
    l = np.array(l)
    img1 = get_images_in_range(i,i+1, list_files, args["data_location"])
    axs[0].imshow(cv2.cvtColor(img1, cv2.COLOR_BGR2RGB))
    a = np.argmin(l)
    img2 = get_images_in_range(a,a+1, list_files, args["data_location"])
    axs[1].imshow(cv2.cvtColor(img2, cv2.COLOR_BGR2RGB))
    plt.title(np.min(l))
    plt.show()
     
image_pairs = []
residuals = []
for i in tqdm(range(len(list_files))):
    residual_local = []
    for j in range(len(list_files_2)):
        img1 = get_images_in_range(i,i+1, list_files, args["data_location"])
        img2 = get_images_in_range(j,j+1, list_files_2, args["data_location"])
        try:
            residual = get_residual(img1, img2, flann)
            residual_local.append(residual)
        except Exception as e:
            print(e)
            residual_local.append(np.inf)
            continue
        if residual < float(args["residual_constant"]):
            image_pairs.append( (i,j))
            print(list_files[i])
            print(list_files_2[j])
            print(residual)
    residuals.append(residual_local)
    if residual_local != []:
        if min(residual_local) < float(args["residual_constant"]):
            show_img(i, residual_local, list_files, args)
        print(min(residual_local))
    else:
        print("FUICK")
with open('./foo.pkl', 'wb') as f:
    pickle.dump(residuals, f)
