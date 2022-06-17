import numpy as np
import open3d as o3d

def display_inlier_outlier(pcd, ind):

    inlier_pcd = pcd.select_by_index(ind)
    outlier_pcd = pcd.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_pcd.paint_uniform_color([1, 0, 0])
    inlier_pcd.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_pcd, outlier_pcd])

def interactive_display(pcd):
 
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points


def zero_filter(pcd): 

    points = pcd.points
    points_array = np.array(points)

    zero_idx = np.where(~points_array.any(axis=1))[0]

    # points_array = points_array[~np.all(points_array == 0, axis=1)]

    # zero_filtered_pcd = o3d.geometry.PointCloud()
    # zero_filtered_pcd.points = o3d.utility.Vector3dVector(points_array)

    return zero_idx


def density_filter(pcd, model): 

    # In the photobooth case, statistical is faster than radius

    if model == 'statistical': 
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.)
    elif model == 'radius': 
        cl, ind = pcd.remove_radius_outlier(nb_points=50, radius=0.02)
    else: 
        cl, ind = pcd, []
        print("choose a removal model between statistical and radius")

    return cl, ind

def read_pcd(fname='../examples/results/pcd/pcd_from_ray_pattern_img13.ply'): 

    pcd = o3d.io.read_point_cloud(fname)

    return pcd

def idx_reorder(ind, zero_idx, n_points): 

    # get index of all the outlier points removed in density filter
    oud = np.delete(np.arange(n_points-len(zero_idx)), ind)
    nonzero_idx = np.delete(np.arange(n_points), zero_idx)
    reordered_oud = nonzero_idx[oud]

    return reordered_oud
            
if __name__ == '__main__' : 

    # pcd = read_pcd('../input/0406/session_1/pcd_shrinked_c0.ply')
    pcd = read_pcd()
    n_points = len(pcd.points)

    zero_idx = zero_filter(pcd)
    points_array = np.array(pcd.points)
    
    zero_filtered_pcd = pcd.select_by_index(zero_idx, invert=True)
    cl, ind = density_filter(zero_filtered_pcd, 'statistical')

    # ind and oud are idx in zero_filtered_pcd, but not in overall pcd, we should reorder them
    outlier_idx = idx_reorder(ind, zero_idx, n_points)

    print("# of zero points", len(zero_idx))
    print("# of outlier points", len(outlier_idx))
    print("# of all points", len(pcd.points))
    print("# of remaining points", len(ind))

    display_inlier_outlier(zero_filtered_pcd, ind)
    # interactive_display(zero_filtered_pcd.select_by_index(ind))

    # combine the filtered zeros and filtered less density points
    filtered_points = np.concatenate((zero_idx, outlier_idx))

    np.save('../examples/results/picked_points/pcd13_filtered_points', filtered_points)
