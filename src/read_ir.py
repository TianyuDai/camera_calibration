import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

HEIGHT, WIDTH = 1024, 1024

def read_pcd(pcd_fname: str):
	return o3d.io.read_point_cloud(pcd_fname)

def read_bin_file(fname: str, dtype=np.uint16, width=WIDTH, height=HEIGHT):
    return np.fromfile(fname, dtype=dtype).reshape(height, width)

def read_ir_image(ir_image_fname: str):
	return read_bin_file(ir_image_fname)

def get_range_compensated_ir_image(ir_image_fname: str, pcd_fname: str, is_return_pcd=True, is_return_ir_img=True, height=HEIGHT, width=WIDTH):
	"""
	read point cloud file name and ir image file name
	and returns the range compensated ir image
	"""
	pcd = read_pcd(pcd_fname)
	ir_img = read_ir_image(ir_image_fname)
	pts_range = np.linalg.norm(np.asarray(pcd.points), axis=-1).reshape(height, width)
	comp_ir_img = ir_img * (pts_range ** 2)
	res_dict = {}
	res_dict['comp_ir_img'] = comp_ir_img
	if is_return_pcd:
		res_dict['pcd'] = pcd
	if is_return_ir_img:
		res_dict['ir_img'] = ir_img
	return res_dict


dataset_dir = '/home/tdai/ADI/tools/camera_calibration/input/photobooth/0406/session_0/'
ir_f = dataset_dir+'c4_ir.bin'
pcd_f = dataset_dir+'c4.pcd'

# height, width = 1024, 1024
# data = np.fromfile(dataset_dir+'c1_ir.bin', dtype=np.uint16).reshape(height, width)

data_comp = get_range_compensated_ir_image(ir_f, pcd_f)

# ir_comp = data_comp['comp_ir_img']
ir_comp = data_comp['ir_img']

ir_crop = ir_comp[192:832, 192:832]
ir_resize = ir_crop[::10, ::10]
# print(np.shape(data_resize))

plt.figure()
# img = plt.imread(data)
# img_cropped = img[80:720, 180:820]
plt.imshow(ir_resize, cmap='gray')
plt.show()
plt.imsave('/home/tdai/ADI/tools/camera_calibration/results/photobooth/depth/c4_ir.png', ir_resize, cmap='gray')
