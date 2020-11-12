import pickle

import matplotlib.pyplot as plt
import numpy as np
import pudb

with open("residual.pkl", "rb") as f:
    residual = pickle.load(f)
pu.db
maxes = []
for l in residual:
    l = np.array(l)
    l = l[np.isfinite(l)]
    plt.plot(l)
    plt.show()
    maxes.append(l.min())
print(maxes)
print(max(maxes))
