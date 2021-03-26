#! /usr/bin/env python

import rospy
import open3d as o3d
import numpy as np
import cv2
from open3d_ros_helper import open3d_ros_helper as orh
import copy
import time
import sys
import tf
import tf2_ros
import geometry_msgs.msg
import sensor_msgs

from functions import *

import rtde_control
import rtde_receive

def broadcaster_scene_view_frame(broadcaster, scene_view_frame, transformation):

    static_transformStamped = geometry_msgs.msg.TransformStamped()
    transformation.header.stamp = rospy.Time.now()
    transformation.child_frame_id = scene_view_frame

    broadcaster.sendTransform(transformation)

def transform_pcd_from_tf_transformation(pcd, tf_transform):

    q = [0.0, 0.0, 0.0, 0.0]
    q[0] = tf_transform.transform.rotation.x
    q[1] = tf_transform.transform.rotation.y
    q[2] = tf_transform.transform.rotation.z
    q[3] = tf_transform.transform.rotation.w
    rot_mat = tf.transformations.quaternion_matrix(q)
    trans = [0.0, 0.0, 0.0]
    trans[0] = tf_transform.transform.translation.x
    trans[1] = tf_transform.transform.translation.y
    trans[2] = tf_transform.transform.translation.z

    T = np.eye(4)
    T[:3, :3] = rot_mat[:3, :3]
    T[:3,3] = trans

    return pcd.transform(T)

if __name__ == "__main__":
 
    rospy.init_node('scene_reconstruction')
    pub = rospy.Publisher("/pose_estimation/scene_reconstructed", sensor_msgs.msg.PointCloud2, queue_size=1)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    depth_frame = "camera_depth_optical_frame"
    tcp_frame = "wrist_3_link"
    base_frame = "base" #IMPORTANT: the base_link is not correct

    rtde_c = rtde_control.RTDEControlInterface("192.168.10.20")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.10.20")

    #init_q = rtde_r.getActualTCPPose()
    #print(init_q)
    #rtde_c.stopScript()

    nb_views = 4
    scene_view = []
    view_frames = []
    pose = []
    pose.append([0.19719292224266746, -0.053347698276854975, 0.4458458092119409, -2.0334190764718203, 2.056142119808624, -0.2840782874269488])
    pose.append([0.43824456892579444, 0.24227909620285548, 0.46318420864958404, 0.03370084992098882, 3.0669104890781944, -0.44672143038415657])
    pose.append([0.45625226335188185, -0.009701326395327205, 0.4887193694816872, 2.095155080034649, -2.125064855095747, -0.19233743849858087])
    pose.append([0.3853246145830555, -0.14514573817502446, 0.49467150764084494, -0.052595898841732866, 3.0670238082574635, 0.6236842121566534])

    print(":: Reconstructing the scene.")

    for i in range(nb_views):
        view_frames.append("view" + str(i) + "_frame")
        rtde_c.moveL(pose[i], 0.2, 0.3)
        time.sleep(2)
        scene_view.append(orh.rospc_to_o3dpc(rospy.wait_for_message("/camera/depth/color/points", sensor_msgs.msg.PointCloud2), remove_nans=True))
        tf_transform_base_view = tfBuffer.lookup_transform(base_frame, depth_frame, rospy.Time())
        broadcaster_scene_view_frame(broadcaster, view_frames[i], tf_transform_base_view)
        print("   View %d recorded" % i)

    rtde_c.moveL(pose[0], 0.2, 0.3)
    rtde_c.stopScript()

    print(":: Concatenating point clouds.")
    scene_reconstructed = scene_view[0]
    for j in range(nb_views-1):
        tf_transform_view_to_view0 = tfBuffer.lookup_transform(view_frames[0], view_frames[j+1], rospy.Time())
        scene_view[j+1] = transform_pcd_from_tf_transformation(scene_view[j+1], tf_transform_view_to_view0)
        scene_reconstructed = scene_reconstructed + scene_view[j+1]

    ## THRESHOLDING PARAMETERS
    # default values (they should be the same because all the view will be refer to the same position)
    ## IDEA: make the region of interest depending on the ARUCO codes
    # roi = rospy.wait_for_message("/aruco/detected_roi", np.array()) we can get the ROI in the 3D space from the topic /aruco/detected_roi
    # then we use this roi to threshold the scene
    z_distance = 0.53
    x_distance = 0.2
    y_distance = 0.15
    distance_threshold = 0.0009

    scene_pcd = threshold_filter_z(scene_reconstructed, z_distance=z_distance)
    scene_pcd = threshold_filter_in_axis(scene_pcd, distance=x_distance, axis=0 ,keep_points_outside_threshold=False)
    scene_pcd = threshold_filter_in_axis(scene_pcd, distance=y_distance, axis=1 ,keep_points_outside_threshold=False, display=True)
    _, scene_pcd = segment_plane_ransac(cloud=scene_pcd, distance_threshold=distance_threshold, ransac_n=3, num_iterations=100, display=True)

    o3d.visualization.draw_geometries([scene_pcd])
    scene_pcd_pointcloud2 = orh.o3dpc_to_rospc(scene_pcd)
    scene_pcd_pointcloud2.header.frame_id = "camera_depth_optical_frame"
    print("Reconstruction DONE! Scene_reconstructed published.")

    scene_pcd_pointcloud2.header.stamp = rospy.Time.now()
    pub.publish(scene_pcd_pointcloud2)
    rospy.sleep(2.0)

    quit()

    '''
    while not rospy.is_shutdown():
        scene_pcd_pointcloud2.header.stamp = rospy.Time.now()
        pub.publish(scene_pcd_pointcloud2)
        rospy.sleep(1.0)
    '''