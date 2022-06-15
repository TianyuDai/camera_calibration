import numpy as np
import open3d as o3d
import PIL

# points_3d_all = np.loadtxt('examples/pcd/points_from_ray_pattern_img13.txt')
# depth_image = o3d.io.read_image("examples/rgp/00013.jpg")
# depth_array = np.array(depth_image)

im = PIL.Image.open('examples/rgb/00013.jpg')
width, height = im.size
# height, width = 1536, 2048
print(height, width)
points_2D = np.loadtxt("examples/picked_points/points_2d.txt")
points_3D = np.loadtxt("examples/picked_points/points_3d_1.txt")

n_points = len(points_2D)
i_point = 0
print(np.shape(points_2D))
print(np.shape(points_3D))
tiepoint_pairs_mat = np.zeros((2*n_points, 12))

np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})
for i in range(n_points): 
    # x_idx = np.random.randint(0, width)
    # y_idx = np.random.randint(0, height)
    # d = depth_array[y_idx][x_idx]
    # if d == 0: 
    #     continue
    i_point_2d = points_2D[i]
    u, v, w = i_point_2d[0] / height, i_point_2d[1] / height, 1.
    # n_points -= 1
    # u, v, w = x_idx / height, y_idx / height, 1.
    i_point_3d = points_3D[i]

    scale = i_point_3d[2]
    # i_point_3d[0] /= scale
    # i_point_3d[1] /= scale
    # i_point_3d[2] /= scale
    i_point_3d = np.array([i_point_3d[0] / scale, i_point_3d[1] / scale, 1., scale]) 
    # i_point_3d = np.append(i_point_3d, scale)
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
    # x_idx = np.random.randint(0, width)
    # y_idx = np.random.randint(0, height)
    # d = depth_array[y_idx][x_idx]
    # if d == 0: 
    #     continue
    # else: 
    flag = False
    i_point_3d = points_3D[0]
    scale = i_point_3d[2] / height
    i_point_3d[0] /= scale
    i_point_3d[1] /= scale
    i_point_3d[2] /= scale 
    i_point_3d = np.append(i_point_3d, scale)
    norm = points_2D[0][0] / np.matmul(P, np.array([i_point_3d]).T)[0][0]

print(P*norm)

np.savetxt('examples/results/camera/depth_to_rgb_proj_mat_img13.txt', norm*P)
