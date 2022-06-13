import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

img = mpimg.imread("examples/rgb/00013.jpg")
fig, ax = plt.subplots()
ax.imshow(img)

points_2d = plt.ginput(n=15, timeout=-1)
np.savetxt('examples/points_2d.txt', points_2d)

for i, point in enumerate(points_2d): 
    ax.scatter(point[0], point[1], s=10, marker='+', color='red', linewidth=0.5)
    ax.text(point[0]+10, point[1]+2, i, color='red', fontsize=15)
plt.savefig("examples/results/00013_labeled.jpg", dpi=200)
