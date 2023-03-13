import numpy as np
import matplotlib.pyplot as plt

omap = np.loadtxt('map.txt')
plt.imshow(omap, origin='lower')
plt.show()