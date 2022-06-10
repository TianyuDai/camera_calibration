import numpy as np
import json
import open3d as o3d

with open("examples/session_17/intrinsic.json") as f: 
    camera_info = json.load(f)
    intrinsic_mat = np.array(camera_info['intrinsic_matrix']).reshape((3, 3)).T

transform_mat = np.concatenate((intrinsic_mat, np.zeros((3, 1))), axis=1)
# transform_mat_inv = np.linalg.inv(transform_mat)
w = height = 1536

points_2d = np.loadtxt("examples/points_2d.txt")
points_2d_homo = np.concatenate((points_2d, np.array([[w]*len(points_2d)]).T), axis=1)


depth_image = o3d.io.read_image("examples/session_17/00000.png")
depth_array = np.array(depth_image)
print(np.shape(depth_array))

fx = intrinsic_mat[0][0]
fy = intrinsic_mat[1][1]
cx = intrinsic_mat[0][2]
cy = intrinsic_mat[1][2]

n_points = len(points_2d)
points_3d = np.zeros((n_points, 3))

np.set_printoptions(threshold=np.inf)
# print(depth_array)

for i, i_point_2d in enumerate(points_2d_homo): 
    u, v, w = [round(r) for r in i_point_2d]
    d = depth_array[v][u]
    # u, v, w = (u-cx)/w, (v-cy)/w, 1
    u, v, w = (u-cx), (v-cy), w
    # print(u, v, w)
    # z = np.sqrt(d**2/(1.+u**2/fx**2+v**2/fy**2))
    z = d
    # print(d**2, u**2/fx**2+v**2/fy**2, z)
    x = u*z/fx
    y = v*z/fy
    i_point_3d = np.array([x, y, z])
    points_3d[i] = i_point_3d
    # print(points_3d)
    # i_point_3d = np.matmul(transform_mat_inv, np.array([i_point_2d]).T)
    # print(i_point_3d)

np.savetxt("examples/points_3d_generated.txt", points_3d)

width = 2048

pcd_points = np.zeros((width*height, 3))
for i in range(width): 
    for j in range(height):
        d = depth_array[j][i]
        if d == 0: 
            continue
        u, v = (i-cx), (j-cy)
        # z = np.sqrt(d**2/(1.+u**2/fx**2+v**2/fy**2))
        z = d
        x = u*z/fx
        y = v*z/fy
        i_point_3d = np.array([x, y, z])
        # print(i, j)
        pcd_points[i*height+j] = i_point_3d

np.savetxt("examples/points_pcd_generated.txt", pcd_points)  
