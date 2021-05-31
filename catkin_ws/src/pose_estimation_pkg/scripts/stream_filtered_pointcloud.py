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

    rospy.init_node('stream_filtered_pointcloud_node')
    wait_for_detected_markers = rospy.wait_for_message("/aruco/img_detected_markers", sensor_msgs.msg.Image, rospy.Duration(20.0))
    pub = rospy.Publisher("/pose_estimation/filtered_pointcloud", sensor_msgs.msg.PointCloud2, queue_size=1)
    scene_pcd = o3d.geometry.PointCloud()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(5.0)

    while not rospy.is_shutdown():
        rate.sleep()

        data = rospy.wait_for_message("/camera/depth/color/points", sensor_msgs.msg.PointCloud2)
        scene_pcd = orh.rospc_to_o3dpc(data, remove_nans=True)

        detected_roi = []
        for i in range(4):
            aruco_id_frame = "depth_aruco_id_" + str(i+1)
            tf_transform = tfBuffer.lookup_transform("camera_depth_optical_frame", aruco_id_frame, rospy.Time(0), rospy.Duration(0.5))
            detected_roi.append((tf_transform.transform.translation.x,
                                tf_transform.transform.translation.y,
                                tf_transform.transform.translation.z))
        size_aruco = 0.04
        detected_roi = np.array(detected_roi)
        x_min, y_min, z_min = np.amin(detected_roi, axis=0)
        x_min_idx, y_min_idx, z_min_idx = np.argmin(detected_roi, axis=0)
        x_max, y_max, z_max = np.amax(detected_roi, axis=0)
        x_max_idx, y_max_idx, z_max_idx = np.argmax(detected_roi, axis=0)
        x_min = x_min - size_aruco/2 
        x_max = x_max + size_aruco/2 
        y_min = y_min - size_aruco/2 
        y_max = y_max + size_aruco/2 
        z_min = z_max - 0.2

        print("x_min = %.3f idx = %d" %(x_min, x_min_idx))
        print("x_max = %.3f idx = %d" %(x_max, x_max_idx))
        print("y_min = %.3f idx = %d" %(y_min, y_min_idx))
        print("y_max = %.3f idx = %d" %(y_max, y_max_idx))
        print("z_min = %.3f" %z_min)
        print("z_max = %.3f" %z_max)

        scene_pcd = threshold_filter_min_max(scene_pcd, axis=0, min_distance=x_min, max_distance=x_max)
        scene_pcd = threshold_filter_min_max(scene_pcd, axis=1, min_distance=y_min, max_distance=y_max)
        scene_pcd = threshold_filter_min_max(scene_pcd, axis=2, min_distance=z_min, max_distance=z_max)

        scene_pcd_pointcloud2 = orh.o3dpc_to_rospc(scene_pcd, frame_id="camera_depth_optical_frame", stamp=rospy.Time.now())
        pub.publish(scene_pcd_pointcloud2)

    quit()
    rospy.spin()
    print("DONE")