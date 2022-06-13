import numpy as np
import open3d as o3d

points_3d_all = np.loadtxt('examples/pcd/points_from_ray_pattern_img13.txt')
depth_image = o3d.io.read_image("examples/depth/00013.png")
depth_array = np.array(depth_image)

height, width = np.shape(depth_array)
print(height, width)

n_points = 30
i_point = 0
tiepoint_pairs_mat = np.zeros((2*n_points, 12))

np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})
while(n_points > 0): 
    x_idx = np.random.randint(0, width)
    y_idx = np.random.randint(0, height)
    d = depth_array[y_idx][x_idx]
    if d == 0: 
        continue
    n_points -= 1
    u, v, w = x_idx / height, y_idx / height, 1.
    i_point_3d = points_3d_all[y_idx*width+x_idx]
    scale = i_point_3d[2]
    i_point_3d[0] /= scale
    i_point_3d[1] /= scale
    i_point_3d[2] /= scale 
    i_point_3d = np.append(i_point_3d, scale)
    for j in range(4): 
        tiepoint_pairs_mat[2*i_point][4+j] = -w * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point][8+j] = v * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point+1][j] = w * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point+1][8+j] = -u * i_point_3d[j]
    i_point += 1

u, s, vh = np.linalg.svd(tiepoint_pairs_mat, full_matrices=True)
np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})
P = vh[-1].reshape(3, 4)

flag = True
while flag: 
    x_idx = np.random.randint(0, width)
    y_idx = np.random.randint(0, height)
    d = depth_array[y_idx][x_idx]
    if d == 0: 
        continue
    else: 
        flag = False
    i_point_3d = points_3d_all[y_idx*width+x_idx]
    scale = i_point_3d[2] / height
    i_point_3d[0] /= scale
    i_point_3d[1] /= scale
    i_point_3d[2] /= scale 
    i_point_3d = np.append(i_point_3d, scale)
    norm = x_idx / np.matmul(P, np.array([i_point_3d]).T)[0][0]

print(P*norm)

np.savetxt('examples/camera_info/proj_mat_img13.txt', norm*P)


