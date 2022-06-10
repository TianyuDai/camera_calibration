# examples/python/visualization/interactive_visualization.py

import numpy as np
import copy
import open3d as o3d
"""
def pick_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()



if __name__ == "__main__":
    demo_crop_geometry()
    demo_manual_registration()
"""

pcd = o3d.io.read_point_cloud("examples/point_cloud.ply")

vis = o3d.visualization.VisualizerWithEditing()
# o3d.visualization.draw_geometries_with_editing([pcd])
vis.create_window()
vis.add_geometry(pcd)
vis.run()  # user picks points
vis.destroy_window()


points = vis.get_picked_points()
# points = vis.PickedPoint()

# vis.capture_screen_image("picked_3d.png", do_render=True)
# print(points.dir())
