import open3d as o3d
import numpy as np
import PIL

im = PIL.Image.open('examples/rgb/00013.jpg')
width, height = im.size
rgb = im.getdata()  # rgb is saved as [0, 1] values

proj_mat = np.loadtxt("examples/camera_info/depth_to_rgb_proj_mat_img13.txt")
pcd = o3d.io.read_point_cloud("examples/pcd/pcd_from_ray_pattern_img13.ply")
points_3D = np.array(pcd.points)

n_points = len(points_3D)
colors = np.zeros((n_points, 3))

for i_point in range(n_points): 
    i_point_3D = points_3D[i_point]
    
    if np.all((i_point_3D == 0)): 
       colors[i_point] = [0, 0, 0]
       continue
    
    scale = i_point_3D[2]
    i_point_3D = np.array([[i_point_3D[0] / scale, i_point_3D[1] / scale, i_point_3D[2] / scale, 1./scale]])
    
    i_point_2D = np.matmul(proj_mat, i_point_3D.T).T[0]
    x_idx, y_idx = min(int(i_point_2D[0] * height), width-1), min(int(i_point_2D[1] * height), height-1)
    i_rgb = [i/255 for i in rgb[y_idx*width+x_idx]] # rgb should be converted to [0, 255] values to colorize point cloud
    colors[i_point] = i_rgb

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points_3D)
pcd.colors = o3d.utility.Vector3dVector(colors)
o3d.visualization.draw_geometries([pcd])

o3d.io.write_point_cloud('examples/results/pcd/pcd_from_ray_pattern_img13_colorized.ply', pcd) 
