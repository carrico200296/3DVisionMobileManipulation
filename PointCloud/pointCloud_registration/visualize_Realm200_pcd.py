import open3d as o3d
import numpy as np
import copy

def draw_registrations(source, target, transformation = None, recolor = False):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    if(recolor):
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
    if(transformation is not None):
        source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


#pcd = o3d.io.read_point_cloud("m200.ply")
#o3d.io.write_point_cloud("m200.pcd", pcd)

pcd = o3d.io.read_point_cloud("pcd_twoM200components_3.pcd")
#pcd.translate((0,0,0), relative=False)

# threshold data
points = np.asarray(pcd.points)
pcd_sel = pcd.select_by_index(np.where(points[:, 2] < 0.53)[0])
pcd_sel.scale(2, center=pcd_sel.get_center())

# visualize different point clouds
#o3d.visualization.draw_geometries([pcd])
o3d.visualization.draw_geometries([pcd_sel])



'''
source = o3d.io.read_point_cloud("m200.pcd")
source.scale(0.01, center=source.get_center())
target = copy.deepcopy(pcd_sel)
target.scale(2, center=target.get_center())
#target = o3d.io.read_point_cloud("m200.pcd")

T = np.eye(4)
T[:3, :3] = target.get_rotation_matrix_from_xyz((0, np.pi / 3, np.pi / 2))
T[0, 3] = 1
T[1, 3] = 1.3
print(T)

source.translate((0,0,0), relative=False)
target.translate((0,0,0), relative=False)
source.transform(T)
print(f'Center of mesh: {source.get_center()}')
print(f'Center of mesh: {target.get_center()}')

# Show models side by side
o3d.visualization.draw_geometries([source, target])


voxel_size = 0.7
source_down = source.voxel_down_sample(voxel_size)
target_down = target.voxel_down_sample(voxel_size)

# Show models side by side
draw_registrations(source_down, target_down, None, True)
'''