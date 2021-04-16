#! /usr/bin/env python

import rospy
import open3d as o3d
import numpy as np
import cv2
from cv2 import aruco
import copy
import time
import sys
import tf
import tf2_ros
import geometry_msgs.msg
import sensor_msgs
import cv_bridge
import matplotlib.pyplot as plt
from open3d_ros_helper import open3d_ros_helper as orh

from functions import *


if __name__ == "__main__":

    rospy.init_node("aruco_detected_roi")
    bridge = cv_bridge.CvBridge()
    camera_type = "D415"

    broadcaster = tf2_ros.TransformBroadcaster()
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    data_cloud = rospy.wait_for_message("/camera/depth/color/points", sensor_msgs.msg.PointCloud2)
    scene_pcd = orh.rospc_to_o3dpc(data_cloud, remove_nans=True)
    rate = rospy.Rate(10)


    while not rospy.is_shutdown():
        rate.sleep()
        data_color = rospy.wait_for_message("/camera/color/image_raw", sensor_msgs.msg.Image)
        color_image = bridge.imgmsg_to_cv2(data_color, desired_encoding="passthrough")

        #data_depth = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", sensor_msgs.msg.Image, timeout=0.2)
        #depth_image = bridge.imgmsg_to_cv2(data_depth, desired_encoding="passthrough")
        #depth_array = np.array(depth_image, dtype=np.float32)

        img_markers, markerCorners, markerIds = detect_aruco_markers(img=color_image, display=False)
        img_axis, rvecs, tvecs = detect_marker_pose(img=color_image, corners=markerCorners, ids=markerIds, camera_type=camera_type, display=False)

        if markerIds is not None:
            for i in range(len(markerIds)):
                #c = markerCorners[i][0]
                #print('id: ', ids[i][0])
                #print('depth: ', depth_array[int(c[:, 1].mean()), int(c[:, 0].mean())])
            
                color_aruco_frame = "color_aruco_id_" + str(markerIds[i][0])
                broadcaster_aruco_color_frame(broadcaster, rvecs[i], tvecs[i], color_aruco_frame)
                transformStamped = tfBuffer.lookup_transform("camera_depth_optical_frame", color_aruco_frame, rospy.Time(), rospy.Duration(0.5))
                depth_aruco_frame = "depth_aruco_id_" + str(markerIds[i][0])
                broadcaster_aruco_depth_frame(broadcaster, transformStamped, depth_aruco_frame)

    rospy.spin()