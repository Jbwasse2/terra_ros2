import pickle

import matplotlib.pyplot as plt
import numpy as np
import pudb

with open("residual.pkl", "rb") as f:
    residual = pickle.load(f)
pu.db
maxes = []
for counter, l in enumerate(residual):
    l = np.array(l)
    l = l[np.isfinite(l)]
    plt.plot(l)
    plt.title(counter)
    plt.show()
    maxes.append(l.min())
print(maxes)
print(max(maxes))
