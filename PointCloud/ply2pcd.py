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

source = o3d.io.read_point_cloud("m200.pcd")
target = o3d.io.read_point_cloud("m200.pcd")

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


voxel_size = 0.7
source_down = source.voxel_down_sample(voxel_size)
target_down = target.voxel_down_sample(voxel_size)

# Show models side by side
draw_registrations(source_down, target_down, None, True)
