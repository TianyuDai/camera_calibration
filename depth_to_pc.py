import open3d as o3d
import json
import glob
import argparse
import numpy as np
import matplotlib.pyplot as plt

with open("examples/session_17/intrinsic.json") as f: 
    camera_info = json.load(f)
    intrinsic_mat = np.array(camera_info['intrinsic_matrix']).reshape((3, 3)).T

# vis=o3d.visualization.VisualizerWithKeyCallback()
# vis.create_window()

# pcd = open3d.geometry.create_point_cloud_from_depth_image("examples/00000.png", intrinsic_mat)
depth_image = o3d.io.read_image("examples/session_17/00000.png")

cam = o3d.camera.PinholeCameraIntrinsic()
cam.intrinsic_matrix = intrinsic_mat

pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_image, cam)
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
# downpcd = pcd.voxel_down_sample(voxel_size=0.05)

"""
o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])
"""

o3d.visualization.draw_geometries([pcd])
o3d.io.write_point_cloud('point_cloud.ply', pcd)
# plt.figure()
# plt.imshow(depth_image)
# plt.show()
"""
vis.add_geometry(pcd, reset_bounding_box=True)
vis.update_renderer()

vis.poll_events()
vis.run()
"""
