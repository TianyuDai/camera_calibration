import numpy as np
import open3d as o3d
import utils

pcd = utils.read_pcd("../examples/results/pcd/pcd_from_ray_pattern_img13.ply")
# pcd = utils.read_pcd("../input/0405/session_2/c0.pcd")

picked_points = utils.interactive_display()

points = np.array(pcd.points)
picked_points_coord = points[picked_points] # get the corresponding coordinates from pcd

# np.savetxt("examples/results/picked_points/points_3d.txt", picked_points_coord)
print(picked_points_coord)
