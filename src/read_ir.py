import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import sys
from PIL import Image

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


img_idx = 0

dataset_dir = '/home/tdai/ADI/tools/camera_calibration/input/photobooth/0406/session_0/'
ir_f = dataset_dir+'c{}_ir.bin'.format(img_idx)
pcd_f = dataset_dir+'c{}.pcd'.format(img_idx)

# height, width = 1024, 1024
# data = np.fromfile(dataset_dir+'c1_ir.bin', dtype=np.uint16).reshape(height, width)

data_comp = get_range_compensated_ir_image(ir_f, pcd_f)

# ir_comp = data_comp['comp_ir_img']
ir_comp = data_comp['ir_img']

ir_crop = ir_comp[80:720, 180:820]


ir_resize = ir_crop[::10, ::10]
# data_comp = get_range_compensated_ir_image(ir_f, pcd_f)
# print(np.shape(data_resize))

mask = o3d.io.read_image('/home/tdai/ADI/tools/camera_calibration/results/photobooth/depth/masks/c{}_mask.png'.format(img_idx))
# mask = o3d.io.read_image('/home/tdai/ADI/tools/camera_calibration/results/photobooth/depth/masks/0000.png')
mask_array = np.array(mask)
# print(mask_array)
# print(np.shape(ir_resize))
plt.imsave('/home/tdai/ADI/tools/camera_calibration/results/photobooth/depth/c{}_ir.png'.format(img_idx), ir_resize, cmap='gray')


# ir_3c = o3d.io.read_image('/home/tdai/ADI/tools/camera_calibration/results/photobooth/depth/c{}_ir.png'.format(img_idx))
ir_3c = o3d.io.read_image(dataset_dir+'0000.png')
ir_3c_array = np.array(ir_3c)
print(np.shape(ir_3c_array))
print(np.max(ir_3c_array), np.min(ir_3c_array))

ir_3c_convert = ir_3c_array[:, :, :3]
for i in range(64): 
    for j in range(64):
        if not mask_array[i, j, 0]:  
            ir_3c_convert[i, j] = np.array([255, 255, 255])

np.set_printoptions(threshold=sys.maxsize)
# print(ir_3c_convert)
plt.figure()
plt.imshow(ir_3c_convert, cmap='gray')
plt.show()
im = Image.fromarray(ir_3c_convert)
im.save('/home/tdai/ADI/tools/camera_calibration/results/photobooth/depth/c{}_ir_convert.png'.format(img_idx))
