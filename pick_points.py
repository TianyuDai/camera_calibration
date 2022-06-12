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

"""
null_points = 0
for i in range(height): 
    for j in range(width): 
        d = depth_array[i][j]
        if d == 0: 
            null_points += 1

print("null ratio: ", null_points / height / width)


null_points = 0
n_points = len(points_3d_all)
for i in range(n_points): 
    if np.all(points_3d_all[i] == 0): 
        null_points += 1

print("null ratio: ", null_points / height / width)
"""

points_2d = np.zeros((height*width, 3))
transform_mat = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])

for i in range(height*width): 
    if np.all(points_3d_all[i] == 0): 
        continue
    # print(np.shape(transform_mat), np.shape(np.array([np.append(points_3d_all[i], 1)]).T))
    points_2d[i] = np.matmul(transform_mat,np.array([np.append(points_3d_all[i], 1)]).T)[0]

n_picked_points = 30
i_point = 0
while(n_picked_points > 0):
    picked_i = np.random.randint(0, height*width) 
    if np.all(points_3d_all[picked_i] == 0):
        continue
    n_picked_points -= 1
    u, v, w = points_2d[picked_i]
    i_point_3d = np.array(np.append(points_3d_all[picked_i], 1))

    for j in range(4): 
        tiepoint_pairs_mat[2*i_point][4+j] = -w * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point][8+j] = v * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point+1][j] = w * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point+1][8+j] = -u * i_point_3d[j]
    i_point += 1
    
u, s, vh = np.linalg.svd(tiepoint_pairs_mat, full_matrices=True)
np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})

print(s)
print(min(s))
# print(np.shape(u), np.shape(s), np.shape(vh))
# print(np.shape(tiepoint_pairs_mat), np.shape(vh))
print(np.transpose(vh))
# print("new array", np.matmul(np.matmul(np.transpose(tiepoint_pairs_mat), tiepoint_pairs_mat), np.transpose(vh)))
"""

# a = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
a = transform_mat
u, s, vh = np.linalg.svd(a)
print(s)
print(vh)
print(np.matmul(np.matmul(np.transpose(a), a), vh))

# transformed_3d = 
while(n_points > 0): 
    x_idx = np.random.randint(0, height)
    y_idx = np.random.randint(0, width)
    d = depth_array[x_idx][y_idx]
    if d == 0: 
        continue
    n_points -= 1
    u, v, w = x_idx / height, y_idx / height, 1.
    i_point_3d = points_3d_all[x_idx*width+y_idx]
    scale = i_point_3d[2]
    i_point_3d[0] /= scale
    i_point_3d[1] /= scale
    i_point_3d[2] = 1.
    i_point_3d = np.append(i_point_3d, 1.)

    for j in range(4): 
        tiepoint_pairs_mat[2*i_point][4+j] = -w * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point][8+j] = v * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point+1][j] = w * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point+1][8+j] = -u * i_point_3d[j]
    i_point += 1

u, s, vh = np.linalg.svd(tiepoint_pairs_mat, full_matrices=True)
print(s)
print(min(s))
"""
