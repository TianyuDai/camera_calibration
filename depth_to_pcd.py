import numpy as np
import argparse
import json
import open3d as o3d

# remove zero points in the point cloud
def clean_pcd(pcd): 

    pcd_array = np.array(pcd.points)
    x_threshold = 9.
    mask = pcd_array[:,0] < x_threshold
    pcd_array = pcd_array[mask]
    pcd_array = pcd_array[~np.all(pcd_array == 0., axis=1)]

    pcd.points = o3d.utility.Vector3dVector(pcd_array)

    return pcd

def pcd_from_ray_pattern(args): 

    ray_pattern = np.load(args.dir+'/camera_info/IR_ray_pattern_Kinect1.npy')

    depth_image = o3d.io.read_image(args.dir+"/depth/{:05d}.png".format(args.image))
    depth_array = np.array(depth_image)

    height, width = np.shape(depth_array)
    points_pcd = np.zeros((height*width, 3))
    print(height, width)
    print(np.shape(ray_pattern), np.shape(depth_array))

    for i in range(height): 
        for j in range(width): 
            r = ray_pattern[i, j]
            cos_rx, cos_ry, cos_rz = r
            # print(cos_rz)
            Z = depth_array[i, j]
            distance = Z / cos_rz
            X = distance * cos_rx
            Y = distance * cos_ry
            points_pcd[i*width+j] = np.array([X, Y, Z])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_pcd)
    o3d.visualization.draw_geometries([pcd])
    

def pcd_from_intrinsic(args): 

    with open(args.dir+'/camera_info/intrinsic_depth.json') as f: 
        camera_info = json.load(f)
        intrinsic_mat = np.array(camera_info['intrinsic_matrix']).reshape((3, 3)).T

    depth_image = o3d.io.read_image(args.dir+"/depth/{:05d}.png".format(args.image))
    print(args.dir+"/depth/{:05d}.png".format(args.image))

    cam = o3d.camera.PinholeCameraIntrinsic()
    cam.intrinsic_matrix = intrinsic_mat

    pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_image, cam)
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    pcd = clean_pcd(pcd)
    
    o3d.io.write_point_cloud(args.dir+'/pcd/pcd_intrinsic.ply', pcd)

    if args.visualize: 
        o3d.visualization.draw_geometries([pcd])
    
if __name__ == '__main__': 
    description = 'tools to build 3D point cloud based on depth image using ray pattern'
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--dir', type=str, default='examples', help='path to data')
    parser.add_argument('--pcd', type=str, default='ray_pattern', help='approach to build pcd, choose from ray_pattern, intrinsic')
    parser.add_argument('--image', type=int, default=0, help='index of the image to be processed')
    parser.add_argument('--visualize', type=bool, default=True, help='whether to visualize the point cloud after generating it')
    
    args = parser.parse_args()
    
    if args.pcd == 'ray_pattern': 
        pcd_from_ray_pattern(args)
    elif args.pcd == 'intrinsic': 
        pcd_from_intrinsic(args)
