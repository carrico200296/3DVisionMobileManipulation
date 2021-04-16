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

if __name__ == "__main__":

    ### IDEA: rospy.wait_for_service() when the node receive a service it will perform the components pose_estimation
    rospy.init_node('stream_pointcloud')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    data = rospy.wait_for_message("/camera/depth/color/points", sensor_msgs.msg.PointCloud2)
    scene_pcd = orh.rospc_to_o3dpc(data, remove_nans=True)

    detected_roi = []
    for i in range(4):
        aruco_id_frame = "depth_aruco_id_" + str(i+1)
        tf_transform = tfBuffer.lookup_transform("camera_depth_optical_frame", aruco_id_frame, rospy.Time(0), rospy.Duration(0.3))
        detected_roi.append((tf_transform.transform.translation.x,
                             tf_transform.transform.translation.y,
                             tf_transform.transform.translation.z))
    size_aruco = 0.04
    x_min = detected_roi[0][0] - size_aruco/2 
    x_max = detected_roi[1][0] + size_aruco/2
    y_min = detected_roi[0][1] - size_aruco/2
    y_max = detected_roi[2][1] + size_aruco/2
    z_max = detected_roi[0][2]
    z_min = z_max - 0.2

    print("x_min = %.3f" %x_min)
    print("x_max = %.3f" %x_max)
    print("y_min = %.3f" %y_min)
    print("y_max = %.3f" %y_max)
    print("z_min = %.3f" %z_min)
    print("z_min = %.3f" %z_max)

    scene_pcd = threshold_filter_min_max(scene_pcd, axis=0, min_distance=x_min, max_distance=x_max)
    scene_pcd = threshold_filter_min_max(scene_pcd, axis=1, min_distance=y_min, max_distance=y_max)
    scene_pcd = threshold_filter_min_max(scene_pcd, axis=2, min_distance=z_min, max_distance=z_max)
    o3d.visualization.draw_geometries([scene_pcd])

    p1 = (detected_roi[0][0] + detected_roi[3][0])/2.0, (detected_roi[0][1] + detected_roi[3][1])/2.0, (detected_roi[0][2] + detected_roi[3][2])/2.0
    p1 = detected_roi[0]
    p2 = detected_roi[1]
    p3 = detected_roi[2]
    points_samples = np.asarray([p1, p2, p3])
    print(points_samples)
    distance_threshold = 0.004
    plane_pcd, pcd = segment_plane_3points(cloud=scene_pcd, points_samples=points_samples, distance_threshold=distance_threshold, display=True)
    plane, scene_pcd = segment_plane_ransac(cloud=scene_pcd, distance_threshold=distance_threshold, ransac_n=5, num_iterations=1000, display=True)

    quit()
    rospy.spin()
    print("DONE")
