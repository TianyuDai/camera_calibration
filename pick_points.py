import numpy as np
import open3d as o3d

points_3d_all = np.loadtxt('examples/pcd/points_from_intrinsic.txt')
depth_image = o3d.io.read_image("examples/depth/00002.png")
depth_array = np.array(depth_image)

height = 576
width = 640 

n_points = 30
i_point = 0
tiepoint_pairs_mat = np.zeros((2*n_points, 12))

while(n_points > 0): 
    x_idx = np.random.randint(0, height)
    y_idx = np.random.randint(0, width)
    d = depth_array[x_idx][y_idx]
    if d == 0: 
        continue
    n_points -= 1
    u, v, w = x_idx / height, y_idx / height, d / height
    i_point_3d = points_3d_all[x_idx*width+y_idx]
    i_point_3d = np.append(i_point_3d, 1.)

    for j in range(4): 
        tiepoint_pairs_mat[2*i_point][4+j] = -w * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point][8+j] = v * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point+1][j] = w * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point+1][8+j] = -u * i_point_3d[j]
    i_point += 1

u, s, vh = np.linalg.svd(tiepoint_pairs_mat, full_matrices=True)
print(s)    
