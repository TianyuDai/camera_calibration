import numpy as np
import open3d as o3d
import utils
import matplotlib.pyplot as plt

# pcd = utils.read_pcd("../examples/results/pcd/pcd_from_ray_pattern_img13.ply")
pcd = utils.read_pcd("../input/photobooth/0406/session_0/c1.pcd")

picked_points = utils.interactive_display(pcd)

points = np.array(pcd.points)
picked_points_coord = points[picked_points] # get the corresponding coordinates from pcd

# np.savetxt("examples/results/picked_points/points_3d.txt", picked_points_coord)
print(picked_points_coord)

plt.figure()
plt.scatter(points[:, 0], points[:, 2])
plt.show()
