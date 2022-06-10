import open3d as o3d
import numpy as np

points_pcd = np.loadtxt("examples/points_pcd_generated.txt")

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points_pcd)
o3d.visualization.draw_geometries([pcd])
# pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

