import numpy as np
import argparse
import json
import open3d as o3d
import matplotlib.pyplot as plt

# remove zero points in the point cloud
def clean_pcd(pcd): 

    pcd_array = np.array(pcd.points)
    x_threshold = 9.
    mask = pcd_array[:,0] < x_threshold
    pcd_array = pcd_array[mask]
    pcd_array = pcd_array[~np.all(pcd_array == 0., axis=1)]

    pcd.points = o3d.utility.Vector3dVector(pcd_array)

    return pcd

def read_depth_info(args):
 
    depth_image = o3d.io.read_image(args.dir+"/depth/{:05d}.png".format(args.image))
    
    if args.pcd == 'ray_pattern': 
        camera_info = np.load(args.dir+'/camera_info/IR_ray_pattern_Kinect1.npy')
    else: 
        with open(args.dir+'/camera_info/intrinsic_depth.json') as f: 
            intrinsics = json.load(f)
            camera_info = np.array(intrinsics['intrinsic_matrix']).reshape((3, 3)).T

    return depth_image, camera_info
        
def show_depth(args): 

    depth_image, camera_info = read_depth_info(args)
    plt.figure()
    plt.imshow(depth_image)
    plt.imsave(args.dir+'/depth/{:05d}_imshow.png'.format(args.image), depth_image)

def pcd_from_intrinsic(args): 

    depth_image, camera_info = read_depth_info(args)
    depth_array = np.array(depth_image)

    height, width = np.shape(depth_array)
    print(height, width)
    fx = camera_info[0][0] / height
    fy = camera_info[1][1] / height
    cx = camera_info[0][2] / height
    cy = camera_info[1][2] / height
    # fx, fy, cx, cy /= height

    pcd_points = np.zeros((width*height, 3))
    for i in range(height): 
        for j in range(width):
            d = depth_array[i][j]
            if d == 0: 
                continue
            u, v = j / height, i / height
            z = d / height
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            i_point_3d = np.array([x, y, z])
            pcd_points[i*width+j] = i_point_3d
            # print(depth_array[i][j], pcd_points[i*width+j][2])
    
    pcd = o3d.geometry.PointCloud()
    # Flip y axis to map image plane to world frame
    pcd.points = o3d.utility.Vector3dVector(pcd_points)
    pcd.transform([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    
    if args.visualize: 
        # pcd.transform([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, -1]])
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(pcd, reset_bounding_box=True)
        opt = vis.get_render_option()
        opt.show_coordinate_frame = True
        vis.run()
        vis.destroy_window()
    
    return pcd, pcd_points
    
def pcd_from_ray_pattern(args): 

    # ray_pattern = np.load(args.dir+'/camera_info/IR_ray_pattern_Kinect1.npy')
    # depth_image = o3d.io.read_image(args.dir+"/depth/{:05d}.png".format(args.image))

    depth_image, ray_pattern = read_depth_info(args)
    depth_array = np.array(depth_image)

    height, width = np.shape(depth_array)
    points_pcd = np.zeros((height*width, 3))

    for i in range(height): 
        for j in range(width): 
            r = ray_pattern[i, j]
            Z = depth_array[i, j]
            X = r[0] * Z / r[2]
            Y = r[1] * Z / r[2]
            points_pcd[i*width+j] = np.array([X, Y, Z])

    pcd = o3d.geometry.PointCloud()
    # Flip y axis to map image plane to world frame
    pcd.points = o3d.utility.Vector3dVector(points_pcd)
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    # o3d.visualization.draw_geometries([pcd])

    if args.visualize: 
        # pcd.transform([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, -1]])
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(pcd, reset_bounding_box=True)
        opt = vis.get_render_option()
        opt.show_coordinate_frame = True
        vis.run()
        vis.destroy_window()
    
    return pcd, points_pcd

def pcd_from_o3d(args): 

    with open(args.dir+'/camera_info/intrinsic_depth.json') as f: 
        camera_info = json.load(f)
        intrinsic_mat = np.array(camera_info['intrinsic_matrix']).reshape((3, 3)).T

    depth_image = o3d.io.read_image(args.dir+"/depth/{:05d}.png".format(args.image))

    cam = o3d.camera.PinholeCameraIntrinsic()
    cam.intrinsic_matrix = intrinsic_mat

    pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_image, cam)

    pcd = clean_pcd(pcd)
    points_pcd = pcd.points
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    # pcd.transform([[0, 0, -1, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
    # o3d.io.write_point_cloud('pcd_from.ply', pcd) 
    
    if args.visualize: 
        # o3d.visualization.draw_geometries([pcd])
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(pcd, reset_bounding_box=True)
        opt = vis.get_render_option()
        opt.show_coordinate_frame = True
        vis.run()
        vis.destroy_window()

    return pcd, points_pcd
    
if __name__ == '__main__': 

    description = 'tools to build 3D point cloud based on depth image using ray pattern'
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--dir', type=str, default='examples', help='path to data')
    parser.add_argument('--pcd', type=str, default='intrinsic', help='approach to build pcd, choose from ray_pattern, intrinsic, open3d')
    parser.add_argument('--image', type=int, default=28, help='index of the image to be processed')
    parser.add_argument('--visualize', type=bool, default=True, help='whether to visualize the point cloud after generating it')
    # parser.add_argument('--points', type=bool, default=True, help='whether to save 3D points information, only valid for ray_pattern and intrinsic approaches')
    
    args = parser.parse_args()

    show_depth(args)
    
    if args.pcd == 'ray_pattern': 
        pcd, points = pcd_from_ray_pattern(args)
        np.savetxt(args.dir+'/pcd/points_from_'+args.pcd+'_img'+str(args.image)+'.txt', points)
    elif args.pcd == 'open3d': 
        pcd, points = pcd_from_o3d(args)
    elif args.pcd == 'intrinsic': 
        pcd, points = pcd_from_intrinsic(args)
        np.savetxt(args.dir+'/pcd/points_from_'+args.pcd+'_img'+str(args.image)+'.txt', points)
    
    o3d.io.write_point_cloud(args.dir+'/pcd/pcd_from_'+args.pcd+'_img'+str(args.image)+'.ply', pcd) 
    # o3d.io.write_point_cloud('pcd_from.ply', pcd) 
