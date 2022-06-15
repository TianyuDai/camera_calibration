import numpy as np
import open3d as o3d

# pcd = o3d.io.read_point_cloud('../input/session_0/c0.pcd')
pcd = o3d.io.read_point_cloud('../examples/results/pcd/pcd_from_ray_pattern_img150.ply')
points_3d = pcd.points

# points_3d = np.loadtxt('../examples/results/pcd/points_from_ray_pattern_img13.txt')
# height, width = 1024, 1024
height, width = 576, 640
# height, width = 1536, 2048

if not height*width == len(points_3d): 
    print("no valid pcd size")

# depth_array = np.fromfile('../input/c0_ir.bin', dtype=np.uint16).reshape(height, width)

# print(np.shape(ir_bin), np.shape(points_3d))
# points_3d = np.loadtxt('../examples/results/pcd/points_from_intrinsic_img0.txt')
# depth_image = o3d.io.read_image("../examples/depth/00000.png")
# depth_array = np.array(depth_image)

# height, width = np.shape(depth_array)

n_points = 30
n_nulls = 0
i_point = 0
tiepoint_pairs_mat = np.zeros((2*n_points, 12))
"""
scaled_points_3d = np.zeros((width*height, 4))

for i, point in enumerate(points_3d): 
    scale = point[2]
    scaled_points_3d[i] = np.array([point[0]/scale, point[1]/scale, 1., 1./scale])

mat = np.array([[0.88, -0.00, 0.56, 0.00], [-0.00, 0.88, 0.58, 0.00], [0.00, 0.00, 1.00, 0.00]])

while (n_points > 0): 
    x_idx = np.random.randint(0, width)
    y_idx = np.random.randint(0, height)
    i_point_3d = points_3d[y_idx*width+x_idx] 
    # if np.all((i_point_3d == 0)):
    if i_point_3d[2] == 0:
        n_nulls += 1 
        continue

    n_points -= 1
    u, v, w = x_idx / height, y_idx / height, 1.
    point_2d = np.array([u, v, w])
    i_point_3d = scaled_points_3d[y_idx*width+x_idx]
    projected = np.matmul(mat, np.array([i_point_3d]).T)
    # point_3d = i_point_3d
    print("2d point", point_2d)
    print("projected 2d point", projected.T[0])

    
    
"""
while(n_points > 0):
 
    x_idx = np.random.randint(0, width)
    y_idx = np.random.randint(0, height)
    i_point_3d = points_3d[y_idx*width+x_idx] 

    if np.all((i_point_3d == 0)) or not np.all((i_point_3d < 3)):
        n_nulls += 1 
        continue

    n_points -= 1
    u, v, w = x_idx / height, y_idx / height, 1.
    # i_point_3d = np.array([p / height for p in points_3d[y_idx*width+x_idx]])
    scale = i_point_3d[2]
    i_point_3d = np.array([p / scale for p in i_point_3d])
    i_point_3d = np.append(i_point_3d, scale)
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

print("number of null points", n_nulls)

flag = True
while flag: 
    x_idx = np.random.randint(0, width)
    y_idx = np.random.randint(0, height)
    i_point_3d = points_3d[y_idx*width+x_idx]
    if np. all((i_point_3d == 0)): 
        continue
    flag = False
    i_point_3d = np.append(i_point_3d, 1.)
    scaled_point = np.array([[i_point_3d[0]/i_point_3d[2], i_point_3d[1]/i_point_3d[2], 1., 1./i_point_3d[2]]])
    norm = x_idx/height / np.matmul(P, scaled_point.T)[0][0]

print("singular values", np.round(s, 2))
print("projection matrix\n", np.round(P*norm, 2))

# np.savetxt('../examples/results/camera/proj_mat_ir_c0.txt', norm*P)

