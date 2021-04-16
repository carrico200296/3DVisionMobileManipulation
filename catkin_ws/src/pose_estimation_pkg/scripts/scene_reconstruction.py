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

if __name__ == "__main__":
 
    rospy.init_node('scene_reconstruction')
    pub = rospy.Publisher("/pose_estimation/scene_reconstructed", sensor_msgs.msg.PointCloud2, queue_size=1)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

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
        time.sleep(1.5)
        #scen_pcd = orh.rospc_to_o3dpc(rospy.wait_for_message("/camera/depth/color/points", sensor_msgs.msg.PointCloud2), remove_nans=True)
        #scene_pcd = get_pcd_roi(scen_pcd, i, tfBuffer)
        #scene_view.append(scen_pcd)
        #o3d.visualization.draw_geometries([scen_pcd])
        scene_view.append(orh.rospc_to_o3dpc(rospy.wait_for_message("/camera/depth/color/points", sensor_msgs.msg.PointCloud2), remove_nans=True))
        tf_transform_base_view = tfBuffer.lookup_transform(base_frame, depth_frame, rospy.Time())
        broadcaster_scene_view_frame(static_broadcaster, view_frames[i], tf_transform_base_view)
        o3d.io.write_point_cloud("scene_view" + str(i) + ".pcd", filter_pcd(scene_view[i]))
        print("   View %d recorded" % i)

    rtde_c.moveL(pose[0], 0.2, 0.3)
    rtde_c.stopScript()

    print(":: Concatenating point clouds.")
    distance_threshold = 0.000009
    scene_view[0] = filter_pcd(scene_view[0])
    scene_reconstructed = scene_view[0]
    _, scene_view[0] = segment_plane_ransac(cloud=scene_view[0], distance_threshold=distance_threshold, ransac_n=3, num_iterations=100, display=False)
    scene_reconstructed_ICP = scene_view[0]

    for j in range(nb_views-1):
        print("--------------------------------------------")
        tf_transform_view_to_view0 = tfBuffer.lookup_transform(view_frames[0], view_frames[j+1], rospy.Time())
        scene_view[j+1] = filter_pcd(scene_view[j+1])
        pcd_view = copy.deepcopy(scene_view[j+1])
        _, scene_view[j+1] = segment_plane_ransac(cloud=scene_view[j+1], distance_threshold=distance_threshold, ransac_n=3, num_iterations=100, display=False)
        scene_view[j+1], pcd_view = tf_transform_pcd_ICP(scene_view[j+1],
                                                scene_view[0],
                                                tf_transform_view_to_view0, pcd_view)
        scene_reconstructed = scene_reconstructed + pcd_view
        scene_reconstructed_ICP = scene_reconstructed_ICP + scene_view[j+1]

    print("Scene Reconstructed no ICP")
    o3d.io.write_point_cloud("scene_reconstructed_m200.pcd", scene_reconstructed)
    o3d.visualization.draw_geometries([scene_reconstructed])
    print("Scene Reconstructed with ICP")
    o3d.io.write_point_cloud("scene_reconstructed_ICP_m200.pcd", scene_reconstructed_ICP)
    o3d.visualization.draw_geometries([scene_reconstructed_ICP])

    scene_pcd = copy.deepcopy(scene_reconstructed_ICP)
    scene_pcd_pointcloud2 = orh.o3dpc_to_rospc(scene_pcd, frame_id="camera_depth_optical_frame", stamp=rospy.Time.now())
    pub.publish(scene_pcd_pointcloud2)
    print("Reconstruction DONE! Scene_reconstructed published.")

    rospy.sleep(2.0)
    quit()