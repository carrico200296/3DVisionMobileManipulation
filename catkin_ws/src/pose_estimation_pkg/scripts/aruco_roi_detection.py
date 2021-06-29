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
import std_msgs
import cv_bridge
import matplotlib.pyplot as plt
from open3d_ros_helper import open3d_ros_helper as orh

from functions import *

import rtde_control
import rtde_receive

if __name__ == "__main__":

    rospy.init_node("aruco_roi_detection_node")
    temp = rospy.wait_for_message("/navigation/mir_status", std_msgs.msg.String, rospy.Duration(300.0))
    print(temp.data)
    pub = rospy.Publisher("/aruco/img_detected_markers", sensor_msgs.msg.Image, queue_size=1)
    bridge = cv_bridge.CvBridge()
    camera_type = "D415"

    # Move to HOME position
    rtde_c = rtde_control.RTDEControlInterface("192.168.12.245")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.12.245")
    home_pose = [0.24629746314958142, 0.39752299707520433, 0.5357813117643294, 2.890757991539826, -1.230076763649183, 1.296199316724078e-05]
    rtde_c.moveL(home_pose, 0.2, 0.5)
    rtde_c.stopScript()
    rtde_c.disconnect()
    time.sleep(1.0)

    broadcaster = tf2_ros.TransformBroadcaster()
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    time.sleep(3.0)
    rate = rospy.Rate(10.0)
    time_start = time.time()
    dt_time = 0.0

    while not rospy.is_shutdown() and dt_time < 30.0:
        rate.sleep()
        data_color = rospy.wait_for_message("/camera/color/image_raw", sensor_msgs.msg.Image)
        color_image = bridge.imgmsg_to_cv2(data_color, desired_encoding="passthrough")
        img_markers, markerCorners, markerIds = detect_aruco_markers(img=color_image, display=False)

        if markerIds is not None:
            img_axis, rvecs, tvecs = detect_marker_pose(img=img_markers, corners=markerCorners, ids=markerIds, camera_type=camera_type, display=False)
            for i in range(len(markerIds)):
                color_aruco_frame = "color_aruco_id_" + str(markerIds[i][0])
                broadcaster_aruco_color_frame(broadcaster, rvecs[i], tvecs[i], color_aruco_frame)
                transformStamped = tfBuffer.lookup_transform("camera_depth_optical_frame", color_aruco_frame, rospy.Time(), rospy.Duration(0.5))
                depth_aruco_frame = "depth_aruco_id_" + str(markerIds[i][0])
                broadcaster_aruco_depth_frame(broadcaster, transformStamped, depth_aruco_frame)
            try:
                pub.publish(bridge.cv2_to_imgmsg(img_axis, "bgr8"))
            except CvBridgeError as e:
                print(e)
        dt_time = time.time() - time_start
    quit()
