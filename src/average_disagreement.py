import numpy as np
import open3d as o3d
import utils

def projected_pcd(depth_image, trans, rot): 
    depth_array = np.array(depth_image)
    height, width = np.shape(depth_array)

    projected_points_3d = np.zeros((width*height, 3))
    rot_inv = np.linalg.inv(rot)
    
    for x in range(width): 
        for y in range(height): 
            u = x / height
            v = y / height
            d = depth_array[y][x] / height

            if d == 0: 
                i_point_3d = np.zeros(3)
                continue
            
            u_, v_, w_ = u-trans[0]/d, v-trans[1]/d, 1.-trans[2]/d
            i_projected_point_3d = np.matmul(rot_inv, np.array([u_, v_, w_]))

            idx_3d = y*width+x
            projected_points_3d[idx_3d] = np.array([p*d for p in i_projected_point_3d])

    return projected_points_3d
            
def Euclidean_dist(projected_points_3d, points_3d, filtered_idx):

    distance = []
    for i in range(len(points_3d)): 
        if i in filtered_idx: 
            continue
        distance.append(np.linalg.norm(points_3d[i] - projected_points_3d[i]))

    return np.mean(distance)

# def display_


if __name__ == '__main__': 

    pcd = utils.read_pcd('../examples/results/pcd/pcd_from_ray_pattern_img13.ply')
    points_3d = np.array(pcd.points)


    depth_image = utils.read_depth('../examples/depth/00013.png')
    
    trans, rot = utils.load_proj_mat("../examples/results/camera/test2.txt")

    projected_points_3d = projected_pcd(depth_image, trans, rot)

    filtered_idx = utils.load_filtered_idx("../examples/results/picked_points/pcd13_filtered_points.npy")
    distance = Euclidean_dist(projected_points_3d, points_3d, filtered_idx)

    print("average distance inlier points is", distance)

    projected_pcd = utils.array_to_pcd(projected_points_3d)
    utils.display_two_pcds(projected_pcd, pcd)
