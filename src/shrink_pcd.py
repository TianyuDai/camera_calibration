import open3d as o3d
import numpy as np
import utils

scale = 1024

# pcd = o3d.io.read_point_cloud('../input/photobooth/0406/session_1/c0.pcd')
pcd = utils.read_pcd('../input/photobooth/0406/session_0/c4.pcd')
points_3d = pcd.points

scaled_points = np.zeros((len(points_3d), 3))

for i, point in enumerate(points_3d): 
    scaled_points[i] = np.array([p/scale for p in point])

pcd.points = o3d.utility.Vector3dVector(scaled_points)

utils.display_pcd(pcd)

o3d.io.write_point_cloud('../input/photobooth/0406/session_0/pcd_shrinked_c4.ply', pcd) 
