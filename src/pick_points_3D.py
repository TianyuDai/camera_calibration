import numpy as np
import open3d as o3d

pcd = o3d.io.read_point_cloud("../examples/results/pcd/pcd_from_ray_pattern_img13.ply")
# pcd = o3d.io.read_point_cloud("../input/0405/session_2/c0.pcd")

vis = o3d.visualization.VisualizerWithEditing()
vis.create_window()
vis.add_geometry(pcd)
vis.run()  # user picks points
# vis.destroy_window()


picked_points = vis.get_picked_points() # get a list of picked points number
points = pcd.points
points = np.array(points)
picked_points_coord = points[picked_points] # get the corresponding coordinates from pcd

# np.savetxt("examples/results/picked_points/points_3d.txt", picked_points_coord)
print(picked_points_coord)
