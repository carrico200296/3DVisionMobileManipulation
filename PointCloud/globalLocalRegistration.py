import open3d as o3d
import numpy as np
import copy
import time

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    print(f'Center of source before transformation: {source_temp.get_center()}')
    print(f'Center of target before transformation: {target_temp.get_center()}')
    # Source in yellow and target in blue
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    print(f'Center of source after transformation: {source_temp.get_center()}')

    frame_source = o3d.geometry.TriangleMesh.create_coordinate_frame(size=40)
    frame_source.translate(source_temp.get_center(),relative=False)
    #R_source = source_temp.get_rotation_matrix_from_xyz()
    #frame_source.rotate(R_source, center=frame_source.get_center())
    #frame_source.scale(10, center=frame_source.get_center())

    frame_target = o3d.geometry.TriangleMesh.create_coordinate_frame(size=40)
    frame_target.translate(target_temp.get_center(), relative=False)
    #R_target = target_temp.get_rotation_matrix_from_xyz()
    #frame_target.rotate(R_target, center=frame_target.get_center())
    #frame_target.scale(10, center=frame_target.get_center())

    o3d.visualization.draw_geometries([source_temp, target_temp, frame_source, frame_target])


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    
    return pcd_down, pcd_fpfh


def prepare_dataset(voxel_size):
    print(":: Load two point clouds and disturb initial pose.")
    #source = o3d.io.read_point_cloud("ICP/r1.pcd")
    #target = o3d.io.read_point_cloud("ICP/r2.pcd")
    source = o3d.io.read_point_cloud("m200.pcd")
    target = o3d.io.read_point_cloud("m200.pcd")

    source.translate((0,0,0), relative=False)
    target.translate((0,0,0), relative=False)

    trans_init = np.eye(4)
    trans_init[:3, :3] = source.get_rotation_matrix_from_xyz((0, np.pi / 3, np.pi / 2))
    trans_init[0, 3] = 150
    trans_init[1, 3] = 30
    source.transform(trans_init)

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),4,
        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
        o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
        o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))
    return result

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" % distance_threshold)
    result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(maximum_correspondence_distance=distance_threshold))
    return result

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result


if __name__ == "__main__":
    voxel_size = 3
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size)
    print("---------------------------------------------------------------------------\n")
    draw_registration_result(source, target, np.identity(4))

    # The Global Registration worked better than the Fast Registration
    start = time.time()
    result_ransac = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)
    print("Global registration took %.3f sec.\n" % (time.time() - start))
    print(result_ransac)
    print("---------------------------------------------------------------------------\n")
    '''
    start = time.time()
    result_ransac = execute_fast_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)
    print("Fast Global registration took %.3f sec.\n" % (time.time() - start))
    print(result_ransac)
    print("---------------------------------------------------------------------------\n")
    '''
    draw_registration_result(source, target, result_ransac.transformation)

    start = time.time()
    result_icp = refine_registration(source, target, source_fpfh, target_fpfh, voxel_size)
    print("Local registration took %.3f sec.\n" % (time.time() - start))
    print(result_icp)
    print("---------------------------------------------------------------------------\n")

    draw_registration_result(source, target, result_icp.transformation)


    print("DONE")

