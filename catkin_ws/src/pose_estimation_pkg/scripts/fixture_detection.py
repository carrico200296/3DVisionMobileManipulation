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
import cv_bridge
import geometry_msgs.msg
import sensor_msgs
import std_msgs
from scipy.spatial.transform import Rotation as R

from functions import *

import rtde_control
import rtde_receive


def place_component(rtde_c, rtde_r, placing_pose, fix_center):
    # Move to placing_pose
    rtde_c.moveL(placing_pose, 0.05, 0.05)
    # Insert component inside the fixture
    insert_pose = rtde_r.getActualTCPPose()
    insert_pose[2] = insert_pose[2] - 0.05
    rtde_c.moveL(insert_pose, 0.01, 0.01)
    # Open gripper
    rtde_c.gripperRelease(0)
    time.sleep(2.0)
    # Move to pick another component
    rtde_c.moveL(placing_pose, 0.1, 0.1)
    end_point = fix_center
    end_point[2] = 0.405
    rtde_c.moveL(end_point, 0.1, 0.1)
    rtde_c.gripperDisable()
    rtde_c.gripperGrasp(0)
    time.sleep(1.0)

def detect_fixture_pose(fixture_model, camera_type, static_broadcaster, tfBuffer):
    bridge = cv_bridge.CvBridge()
    data_color = rospy.wait_for_message("/camera/color/image_raw", sensor_msgs.msg.Image)
    color_image = bridge.imgmsg_to_cv2(data_color, desired_encoding="passthrough")
    img_markers, markerCorners, markerIds = detect_aruco_markers(img=color_image, display=False)

    if markerIds is not None:
        img_axis, rvecs, tvecs = detect_marker_pose(img=img_markers, corners=markerCorners, ids=markerIds, camera_type=camera_type, display=False)
        
        for i in range(len(markerIds)):
            if str(markerIds[i][0]) == fixture_model:
                color_aruco_frame = "color_aruco_id_" + str(markerIds[i][0])
                broadcaster_aruco_color_frame(static_broadcaster, rvecs[i], tvecs[i], color_aruco_frame)
                transformStamped = tfBuffer.lookup_transform("camera_depth_optical_frame", color_aruco_frame, rospy.Time(), rospy.Duration(0.5))
                depth_aruco_frame = "depth_aruco_id_" + str(markerIds[i][0])
                broadcaster_aruco_depth_frame(static_broadcaster, transformStamped, depth_aruco_frame)


if __name__ == "__main__":
 
    rospy.init_node('fixture_detection_node')
    base_frame = "base"

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    broadcaster = tf2_ros.TransformBroadcaster()
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    rtde_c = rtde_control.RTDEControlInterface("192.168.12.245")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.12.245")

    #ARUCO-FIXTURE OFFSET CALCULATION (to calculate statistics)
    #sample_1 = [0.7006637307671114, 0.1018380679219168, 0.2845, 2.908, -1.188, 0.0]
    #sample_2 = [0.7007150217732067, 0.1024004188919436, 0.2845, 2.908, -1.188, 0.0]
    #sample_3 = [0.7009458456988304, 0.10214223936068872, 0.0005296980402354767]
    #sample_4 = [0.7010068766864602, 0.1026414759430313, 0.003968304070235351]
    # fixture_location_manual = [0.7422676686207003, 0.06185000753060148, 0.2779644938213514, 2.9080196434959547, -1.1879900911879482, 2.2863347736277715e-05]

    #>>> diff_x = fixture_location_manual[0] - sample_2[0]
    #>>> print(diff_x)
    #0.0415526468475
    #>>> diff_y = fixture_location_manual[1] - sample_2[1]
    #>>> print(diff_y)
    #-0.0405504113613
    x_fixture_offset = 0.0415526468475
    y_fixture_offset = -0.0405504113613
    height_fixture = 0.150

    #FIXTURE_CENTER - PLACING ORIENTED POSE OFFSET CALCULATION
    #center_fixture_oriented = [0.742260932217375, 0.06183793596849685, 0.22860847605029044, -1.2878227766872377, 1.0705819504509524, -1.074506410798968]
    #center_fixture_offset_oriented = [0.6249967345976565, 0.03867814130236337, 0.2286438278375378, -1.287808247339734, 1.070439178974699, -1.074470886939796]
    
    #>>> diff_x = center_fixture_offset_oriented[0] - center_fixture_oriented[0] 
    #>>> print(diff_x)
    #-0.11726419762
    #>>> diff_y = center_fixture_offset_oriented[1] - center_fixture_oriented[1]
    #>>> print(diff_y)
    #-0.0231597946661

    # PLACING_POSE CALCULATED with VISION vs MANUAL (to calculate statistics) - related with the aruco marker detection samples
    # placing_pose_calculated_sample_3 = [0.6252342949263305, 0.03843203333328873, 0.228, -1.287808247339734, 1.070439178974699, -1.074470886939796]
    # placing_pose_calculated_sample_4 = [0.6252953259139603, 0.038931269915631304, 0.228, -1.287808247339734, 1.070439178974699, -1.074470886939796]
    # placing_pose_manual = [0.6249967345976565, 0.03867814130236337, 0.2286438278375378, -1.287808247339734, 1.070439178974699, -1.074470886939796]

    x_placing_offset = -0.11726419762
    y_placing_offset = -0.0231597946661
    z_placing_offset = 0.228 # it is a good distance

    tool_offset = 0.1245
    z_picking_offset = 0.130 #0.1315 before, all the calculations have been done with 0.130
    z_offset_center_fixture = tool_offset + height_fixture + 0.01

    # Pick a component
    #entry_point_assembly_line = [0.5646680489343061, -0.14747084956424256, 0.1923333064137298, 1.2084273498616414, -2.8987764754231495, 0.0002400822177717899]
    entry_point = [0.40166859802430244, 0.31179738856044764, 0.14938715474534492, 2.9071470273265487, -1.186809924863102, 0.00028546423748938507]
    rtde_c.moveL(entry_point, 0.1, 0.1)
    rtde_c.gripperDisable()
    rtde_c.gripperRelease(0)
    #picking_point_assembly_line  = [0.564682488566143, -0.14747767132023368, 0.16774064035158953, 1.208439480512014, -2.8988074757401505, 0.00017254328108733615]
    picking_point = rtde_r.getActualTCPPose()
    picking_point[2] = z_picking_offset
    rtde_c.moveL(picking_point, 0.1, 0.1)
    rtde_c.gripperGrasp(0)
    time.sleep(2.0)
    rtde_c.moveL(entry_point, 0.2, 0.2)

    # Localize Fixture - take_image_pose: predefined_pose for taking the image, accuracy based on MIR docking position
    take_image_pose = [0.5967660895688741, 0.1253897292211242, 0.47289248807659473, -1.845352220142729, 2.542406034245773, 1.789893252155485e-05]
    rtde_c.moveL(take_image_pose, 0.5, 0.8)
    time.sleep(0.2)
    detect_fixture_pose(fixture_model="200", camera_type="D415", static_broadcaster=static_broadcaster, tfBuffer=tfBuffer)
    tf_base_fixture200 = tfBuffer.lookup_transform(base_frame, "depth_aruco_id_200", rospy.Time(0), rospy.Duration(0.5))
    t, rot_vector = get_t_rotvector_target(tf_transform_base_target=tf_base_fixture200)
    time.sleep(1.0)
    print("Aruco marker position: ")
    print(t)
    fix_center = [t[0] + x_fixture_offset, t[1] + y_fixture_offset, z_offset_center_fixture, 2.908, -1.188, 0.0]
    print("Fixture center: ")
    print(fix_center)
    placing_pose = [fix_center[0] + x_placing_offset, fix_center[1] + y_placing_offset, z_placing_offset, -1.287808247339734, 1.070439178974699, -1.074470886939796]
    print("Placing pose calculated: ")
    print(placing_pose)
    print("Placing pose manual: ")
    center_fixture_offset_oriented = [0.6249967345976565, 0.03867814130236337, 0.2286438278375378, -1.287808247339734, 1.070439178974699, -1.074470886939796]
    print(center_fixture_offset_oriented)

    # Place component
    place_component(rtde_c=rtde_c, rtde_r=rtde_r, placing_pose = placing_pose, fix_center = fix_center)

    rtde_c.gripperDisable()
    rtde_c.stopScript()
    rtde_c.disconnect()

    quit()
    rospy.spin()