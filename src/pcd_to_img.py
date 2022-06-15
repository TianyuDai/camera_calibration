import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud('../examples/results/pcd/pcd_from_ray_pattern_img13.ply')
pcd_2 = o3d.io.read_point_cloud('../examples/results/pcd/pcd_from_intrinsic_img13.ply')
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd, reset_bounding_box=True)
vis.add_geometry(pcd_2, reset_bounding_box=True)
opt = vis.get_render_option()
opt.show_coordinate_frame = True
vis.run()
vis.destroy_window()
