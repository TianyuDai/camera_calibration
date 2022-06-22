import sys
import numpy as np
import open3d as o3d
import utils

pcd = utils.read_pcd('../input/photobooth/0406/session_0/c0.pcd')
# pcd = o3d.io.read_point_cloud('../examples/results/pcd/pcd_from_ray_pattern_img13.ply')
points_3d = pcd.points

# points_3d = np.loadtxt('../examples/results/pcd/points_from_ray_pattern_img13.txt')
height, width = 1024, 1024
# height, width = 576, 640
# height, width = 1536, 2048

if not height*width == len(points_3d): 
    sys.exit("no valid pcd size")

# depth_array = np.fromfile('../input/c0_ir.bin', dtype=np.uint16).reshape(height, width)

# print(np.shape(ir_bin), np.shape(points_3d))
# points_3d = np.loadtxt('../examples/results/pcd/points_from_intrinsic_img0.txt')
# depth_image = o3d.io.read_image("../examples/depth/00000.png")
# depth_array = np.array(depth_image)

# height, width = np.shape(depth_array)

n_points = 1000
i_point = 0
tiepoint_pairs_mat = np.zeros((2*n_points, 12))
filtered_points = np.load("../results/photobooth/picked_points/session_0/c0_filtered_points.npy")

while(n_points > 0):

    # x_idx = np.random.randint(0, width)
    # y_idx = np.random.randint(0, height)
    # sample points around the center to avoid distortion affects
    x_idx = np.random.randint(192, 832)
    y_idx = np.random.randint(192, 832)


    if y_idx * width + x_idx in filtered_points:
        continue

    i_point_3d = points_3d[y_idx*width+x_idx]
    i_point_3d = np.array([p/height for p in i_point_3d])
    # print(i_point_3d) 

    # if np.all((i_point_3d == 0)) or not np.all((i_point_3d < 3.)):
    #     n_nulls += 1 
    #     continue

    n_points -= 1
    # u, v, w = x_idx / height, y_idx / height, 1.
    u, v, w = (x_idx - 192) / 640, (y_idx - 192) / 640, 1.
    # i_point_3d = np.array([p / height for p in points_3d[y_idx*width+x_idx]])
    scale = i_point_3d[2]
    i_point_3d = np.array([p / scale for p in i_point_3d])
    i_point_3d = np.append(i_point_3d, 1./scale)
    # print(i_point_3d)

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
    i_point_3d = points_3d[y_idx*width+x_idx]

    if y_idx*width+x_idx in filtered_points:
        continue

    flag = False
    scale = i_point_3d[2]
    i_point_3d = np.array([p / scale for p in i_point_3d])
    i_point_3d = np.append(i_point_3d, 1./scale)
    # i_point_3d = np.append(i_point_3d, 1.)
    # scaled_point = np.array([[i_point_3d[0]/i_point_3d[2], i_point_3d[1]/i_point_3d[2], 1., 1./i_point_3d[2]]])
    norm = 1. / np.matmul(P, np.array(i_point_3d).T)[2]

print("singular values", np.round(s, 6))
print("projection matrix\n", np.round(P*norm, 6))

np.savetxt('../results/photobooth/camera/session_0/proj_mat_c0.txt', norm*P)

