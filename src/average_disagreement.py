import numpy as np
import open3d as o3d

pcd = o3d.io.read_point_cloud('../examples/results/pcd/pcd_from_ray_pattern_img13.ply')
points_3d = np.array(pcd.points)

depth_image = o3d.io.read_image("../examples/depth/00013.png")
depth_array = np.array(depth_image)
height, width = np.shape(depth_array)

proj_mat = np.loadtxt("../examples/results/camera/test2.txt")

trans = proj_mat.T[-1]
rot = np.delete(proj_mat, -1, 1)
rot_inv = np.linalg.inv(rot)

projected_points_3d = np.zeros((width*height, 3))

distance = []
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
        # projected_points_3d[idx_3d] = i_projected_point_3d
        projected_points_3d[idx_3d] = np.array([p*d for p in i_projected_point_3d])
        
        scale = points_3d[idx_3d][2] / d
        points_3d[idx_3d] = np.array([p/scale for p in points_3d[idx_3d]])

        distance.append(np.linalg.norm(points_3d[idx_3d] - projected_points_3d[idx_3d]))

# ave_dist = np.mean([np.linalg.norm(points_3d[i] - projected_points_3d[i]) for i in range(height*width)])

print("average distance of all non-zero point is", np.mean(distance))

filtered_idx = np.load("../examples/results/picked_points/pcd13_filtered_points.npy")

distance = []
for i in range(height*width): 
    if i in filtered_idx: 
        continue
    distance.append(np.linalg.norm(points_3d[i] - projected_points_3d[i]))

print("average distance of inlier points is", np.mean(distance))
