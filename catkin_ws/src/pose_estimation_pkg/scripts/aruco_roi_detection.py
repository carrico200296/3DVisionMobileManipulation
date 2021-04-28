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

    rospy.init_node("aruco_roi_detection_node")
    pub = rospy.Publisher("/aruco/img_detected_markers", sensor_msgs.msg.Image, queue_size=1)
    bridge = cv_bridge.CvBridge()
    camera_type = "D415"

    broadcaster = tf2_ros.TransformBroadcaster()
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10)
    time.sleep(8.0) # wait 5 seconds until the realsense is initialized 

    while not rospy.is_shutdown():
        rate.sleep()
        data_color = rospy.wait_for_message("/camera/color/image_raw", sensor_msgs.msg.Image)
        color_image = bridge.imgmsg_to_cv2(data_color, desired_encoding="passthrough")

        img_markers, markerCorners, markerIds = detect_aruco_markers(img=color_image, display=False)
        if markerIds is not None:
            img_axis, rvecs, tvecs = detect_marker_pose(img=color_image, corners=markerCorners, ids=markerIds, camera_type=camera_type, display=False)
            for i in range(len(markerIds)):
                color_aruco_frame = "color_aruco_id_" + str(markerIds[i][0])
                broadcaster_aruco_color_frame(broadcaster, rvecs[i], tvecs[i], color_aruco_frame)
                transformStamped = tfBuffer.lookup_transform("camera_depth_optical_frame", color_aruco_frame, rospy.Time(), rospy.Duration(0.5))
                depth_aruco_frame = "depth_aruco_id_" + str(markerIds[i][0])
                broadcaster_aruco_depth_frame(broadcaster, transformStamped, depth_aruco_frame)
            try:
                pub.publish(bridge.cv2_to_imgmsg(img_markers, "bgr8"))
            except CvBridgeError as e:
                print(e)
    rospy.spin()