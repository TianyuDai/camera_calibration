import numpy as np

points_2d = np.loadtxt("examples/points_2d.txt")
points_3d = np.loadtxt("examples/points_3d_generated.txt")

n_points = len(points_2d)
height = 1536
w = height 
tiepoint_pairs_mat = np.zeros((2*n_points, 12))

for i_point in range(n_points): 
    u, v = points_2d[i_point]
    # print(u, v, points_3d[i_point])
    i_point_3d = np.append(points_3d[i_point], 1.)
    # print(i_point_3d)
    for j in range(4): 
        tiepoint_pairs_mat[2*i_point][4+j] = -w * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point][8+j] = v * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point+1][j] = w * i_point_3d[j]
        tiepoint_pairs_mat[2*i_point+1][8+j] = -u * i_point_3d[j]

# print(tiepoint_pairs_mat)
u, s, vh = np.linalg.svd(tiepoint_pairs_mat, full_matrices=True)
print(min(s))
