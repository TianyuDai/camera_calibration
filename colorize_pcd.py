import open3d as o3d
import numpy as np
import PIL

im = PIL.Image.open('examples/rgb/00013.jpg')
width, height = im.size
print(width, height)
# rgb = np.array(im.getdata())
rgb = im.getdata()
# print(max(rgb), min(rgb))

# pixelMap = im.load()
# img = PIL.Image.new( im.mode, im.size)
# pixelsNew = img.load()
"""
for i in range(img.size[0]):
    for j in range(img.size[1]):
        if 205 in pixelMap[i,j]:
            pixelMap[i,j] = (0,0,0,255)
        else:
            pixelsNew[i,j] = pixelMap[i,j]
"""
"""
# img.show()
print(len(rgb))
img_new = PIL.Image.new(mode="RGB", size=(width, int(height/2)))
rgb_new = []
for i, i_rgb in enumerate(rgb): 
    if i >= width*height/2: 
        break
    rgb_new.append(i_rgb)
print(len(rgb_new), height*width/2)
img_new.putdata(rgb_new)
img_new.show()
"""
# print(np.shape(rgb))
# x = img.load()
# int(img.size)
# print(rgb)

# points_3D = np.loadtxt("examples/pcd/points_from_intrinsic_img0.txt")
# depth_image = o3d.io.read_image("examples/depth/00028.png")
# depth_array = np.array(depth_image)

proj_mat = np.loadtxt("examples/camera_info/depth_to_rgb_proj_mat_img13.txt")
pcd = o3d.io.read_point_cloud("examples/pcd/pcd_from_ray_pattern_img13.ply")
points_3D = np.array(pcd.points)

n_points = len(points_3D)
colors = np.zeros((n_points, 3))
# x_list = []
# y_list = []

# points_3D_backup = points_3D
# print(np.shape(points_3D))

for i_point in range(n_points): 
    i_point_3D = points_3D[i_point]
    
    if np.all((i_point_3D == 0)): 
       colors[i_point] = [0, 0, 0]
       continue
    
    scale = i_point_3D[2]
    # print(scale)
    i_point_3D = np.array([[i_point_3D[0] / scale, i_point_3D[1] / scale, i_point_3D[2] / scale, 1./scale]])
    
    i_point_2D = np.matmul(proj_mat, i_point_3D.T).T[0]
    x_idx, y_idx = min(int(i_point_2D[0] * height), width-1), min(int(i_point_2D[1] * height), height-1)
    # print(x_idx, y_idx)
    # x_list.append(x_idx)
    # y_list.append(y_idx)
    # print(x_idx, y_idx, y_idx*width+x_idx)
    i_rgb = [i/255 for i in rgb[y_idx*width+x_idx]]
    # i_rgb = [0.5, 0.5, 0.5]
    colors[i_point] = i_rgb
    
# print(min(x_list), max(x_list), min(y_list), max(y_list))


pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points_3D)
pcd.colors = o3d.utility.Vector3dVector(colors)
o3d.visualization.draw_geometries([pcd])

o3d.io.write_point_cloud('examples/pcd/pcd_from_ray_pattern_img13_colorized.ply', pcd) 
