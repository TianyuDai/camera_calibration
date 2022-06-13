import numpy as np
import open3d as o3d

points_3d_all = np.loadtxt('examples/pcd/points_from_intrinsic_img28.txt')
depth_image = o3d.io.read_image("examples/depth/00028.png")
depth_array = np.array(depth_image)

height, width = np.shape(depth_array)
print(height, width)

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


points_2d = np.zeros((height*width, 3))
transform_mat = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])

for i in range(height*width): 
    if np.all(points_3d_all[i] == 0): 
        continue
    # print(np.shape(transform_mat), np.shape(np.array([np.append(points_3d_all[i], 1)]).T))
    # print(np.array([np.append(points_3d_all[i], 1)]).T)
    points_2d[i] = np.matmul(transform_mat,np.array([np.append(points_3d_all[i], 1)]).T).T[0]
    # print(np.shape(np.matmul(transform_mat,np.array([np.append(points_3d_all[i], 1)]).T))

n_picked_points = 30
i_point = 0
while(n_picked_points > 0):
    picked_i = np.random.randint(0, height*width) 
    if np.all(points_3d_all[picked_i] == 0):
        continue
    n_picked_points -= 1
    u, v, w = points_2d[picked_i]
    # print(u, v, w)
    # u = points_2d[picked_i][0]
    # v = points_2d[picked_i][1]
    # w = points_2d[picked_i][2]
    i_point_3d = np.array(np.append(points_3d_all[picked_i], 1))
    # print(picked_i, points_2d[picked_i], i_point_3d)

    for j in range(4): 
        tiepoint_pairs_mat[2*i_point][4+j] = -w * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point][8+j] = v * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point+1][j] = w * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point+1][8+j] = -u * i_point_3d[j]
    i_point += 1

u, s, vh = np.linalg.svd(tiepoint_pairs_mat, full_matrices=True)
np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})
v = np.transpose(vh)
# print(s)
# print(v)
# print(np.matmul(np.matmul(np.transpose(tiepoint_pairs_mat), tiepoint_pairs_mat), v))
# print(np.matmul(np.square(np.diag(s)), v))
# print(tiepoint_pairs_mat)
s_arr = np.diag(s)
s_arr.resize((len(u), len(v)))
# print(np.matmul(np.matmul(u, s_arr), vh))
# print(np.matmul(tiepoint_pairs_mat, np.array([v[-1]]).T))
# print(min(s))
# print(np.shape(u), np.shape(s), np.shape(vh))
# print(np.shape(tiepoint_pairs_mat), np.shape(vh))
# print(np.transpose(vh))
# print(np.matmul(np.matmul(np.transpose(tiepoint_pairs_mat), tiepoint_pairs_mat), v))
# print(np.matmul(np.matmul(v, np.transpose(s_arr)), s_arr))
# print(np.matmul(np.transpose(s_arr), s_arr))
# print(np.matmul(tiepoint_pairs_mat, np.array([vh[-4]]).T))
# print(vh[-1], vh[-2], vh[-3], vh[-4])
P = vh[-4].reshape(3, 4)
print(P)
# print(vh[-4].reshape(3, 4))
# print(np.matmul(u, s_arr))


a = np.array([[1, 2, 3, 4], [2, 1, 3, 1], [2, 2, 1, 1], [1, 2, 0, 1]])
# a = transform_mat
u, s, vh = np.linalg.svd(a)
v = np.transpose(vh)

# print(a)
# print(np.matmul(np.matmul(u, np.diag(s)), vh))
print(np.matmul(np.matmul(np.transpose(a), a), v))
# print(np.matmul(np.transpose(a), a))
# print(np.matmul(np.matmul(v, np.square(np.diag(s))), vh))
print(np.matmul(v, np.square(np.diag(s))))
# print(s)
# print(v)

# transformed_3d = 
# height = 1024

cx = 514.497314453125
cy = 516.5053100585938
fx = 505.02545166015625
fy = 505.0204772949219
"""

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
    # print(i_point_3d[2], d)
    i_point_3d[0] /= scale
    i_point_3d[1] /= scale
    i_point_3d[2] /= scale 
    i_point_3d = np.append(i_point_3d, scale)
    # print((u-cx)*w/fx, (v-cy)*w/fy, w)
    # print(i_point_3d)
    for j in range(4): 
        tiepoint_pairs_mat[2*i_point][4+j] = -w * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point][8+j] = v * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point+1][j] = w * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point+1][8+j] = -u * i_point_3d[j]
    i_point += 1

u, s, vh = np.linalg.svd(tiepoint_pairs_mat, full_matrices=True)
np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})
print(s)
# print(np.matmul(np.array([vh[-1]]), np.array([vh[-1]]).T))
# print(min(s))
P = vh[-1].reshape(3, 4)
print(P)

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
    print(i_point_3d[0], (x_idx-cx)*height/fx)
    print(x_idx, np.matmul(P, np.array([i_point_3d]).T))
    norm = x_idx / np.matmul(P, np.array([i_point_3d]).T)[0][0]

print(P*norm)

# print(np.matmul(tiepoint_pairs_mat, np.array([vh[-1]]).T))

