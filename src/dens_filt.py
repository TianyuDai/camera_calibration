import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import utils
from PIL import Image
import sys

def display_inlier_outlier(pcd, ind):

    inlier_pcd = pcd.select_by_index(ind)
    outlier_pcd = pcd.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    utils.display_two_pcds(outlier_pcd, inlier_pcd)

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
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.)
    elif model == 'radius': 
        cl, ind = pcd.remove_radius_outlier(nb_points=50, radius=0.02)
    else: 
        cl, ind = pcd, []
        print("choose a removal model between statistical and radius")

    return cl, ind

def idx_reorder(ind, zero_idx, n_points): 

    # get index of all the outlier points removed in density filter
    oud = np.delete(np.arange(n_points-len(zero_idx)), ind)
    nonzero_idx = np.delete(np.arange(n_points), zero_idx)
    reordered_oud = nonzero_idx[oud]

    return reordered_oud
 
def generate_mask(zero_idx):
 
    height, width = 1024, 1024
    mask = (np.ones((height, width, 3)) * 255).astype(np.uint8)

    for idx in zero_idx: 
        x, y = idx % width, idx // width
        mask[y, x] = np.array([0, 0, 0])

    return mask
           
if __name__ == '__main__' : 

    img_idx = 0

    pcd = utils.read_pcd('../input/photobooth/0406/session_0/pcd_shrinked_c{}.ply'.format(img_idx))
    # pcd = read_pcd('../examples/results/pcd/pcd_from_ray_pattern_img13.ply')
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

    # display_inlier_outlier(zero_filtered_pcd, ind)
    # utils.interactive_display(zero_filtered_pcd.select_by_index(ind))

    # combine the filtered zeros and filtered less density points
    filtered_points = np.concatenate((zero_idx, outlier_idx))

    np.save('../results/photobooth/picked_points/session_0/c{}_filtered_points'.format(img_idx), filtered_points)

    mask =  generate_mask(filtered_points)
    mask_crop = mask[80:720, 180:820, :]
    mask_resize = mask_crop[::10, ::10, :]
    print(np.shape(mask_resize))
    print(np.max(mask_resize))
    im = Image.fromarray(mask_resize)
    im.save('../results/photobooth/depth/c{}_mask.png'.format(img_idx))
    # plt.imsave('../results/photobooth/depth/c{}_mask.png'.format(img_idx), mask_resize, cmap='gray')

