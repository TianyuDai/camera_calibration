import numpy as np
import open3d as o3d

def read_pcd(fname): 
    pcd = o3d.io.read_point_cloud(fname)
    return pcd

def read_depth(fname): 
    depth_image = o3d.io.read_image(fname)
    return depth_image

def load_proj_mat(fname): 
    proj_mat = np.loadtxt("../examples/results/camera/test0.txt")

    trans = proj_mat.T[-1]
    rot = np.delete(proj_mat, -1, 1)

    return trans, rot

def load_filtered_idx(fname): 
    filtered_idx = np.load(fname)
    return filtered_idx

def array_to_pcd(array): 
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(array)
    return pcd
    
def display_two_pcds(pcd_1, pcd_2):

    # The first pcd is shown in red 
    pcd_1.paint_uniform_color([1, 0, 0])
    # The second pcd is shown in gray
    pcd_2.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([pcd_1, pcd_2])

def display_pcd(pcd): 
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd, reset_bounding_box=True)
    opt = vis.get_render_option()
    opt.show_coordinate_frame = True
    vis.run()
    vis.destroy_window()

    
def interactive_display(pcd):
 
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()

    picked_points = vis.get_picked_points() # get a list of picked points number
    return picked_points
    

