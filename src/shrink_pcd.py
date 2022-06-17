import open3d as o3d
import numpy as np

scale = 1024

pcd = o3d.io.read_point_cloud('../input/session_0/c0.pcd')
points_3d = pcd.points

scaled_points = np.zeros((len(points_3d), 3))

for i, point in enumerate(points_3d): 
    scaled_points[i] = np.array([p/scale for p in point])

pcd.points = o3d.utility.Vector3dVector(scaled_points)

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd, reset_bounding_box=True)
opt = vis.get_render_option()
opt.show_coordinate_frame = True
vis.run()
vis.destroy_window()

o3d.io.write_point_cloud('../input/session_0/pcd_shrinked_c0.ply', pcd) 
