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
import xlsxwriter

def pick_component(rtde_c, rtde_r, t, rot_vector, tool_offset, z_picking_offset):
    off_set = tool_offset + 0.08
    # Move to the picking-entry pose
    picking_entry_pose = [t[0], t[1], t[2] + off_set, rot_vector[0], rot_vector[1], rot_vector[2]]
    rtde_c.moveL(picking_entry_pose, 0.2, 0.2)
    # Open gripper
    rtde_c.gripperDisable()
    rtde_c.gripperRelease(0)
    # Move to picking_pose (z = 0.130 for m200 component)
    picking_pose = rtde_r.getActualTCPPose()
    picking_pose[2] = z_picking_offset
    print("Component picking Point:")
    print(picking_pose)
    rtde_c.moveL(picking_pose, 0.1, 0.1)
    # Close gripper
    rtde_c.gripperGrasp(0)
    time.sleep(1.5)
    # Move to picking-entry pose
    rtde_c.moveL(picking_entry_pose, 0.2, 0.2)

if __name__ == "__main__":

    file_time = str(int(time.time()))
    file_name = '/home/carlos/Desktop/samples_for_report/results/test_picking/sample_data_' + file_time + '.xlsx' 
    workbook = xlsxwriter.Workbook(file_name)
    worksheet = workbook.add_worksheet()
    row = 0
    column = 0
    headers = ["Sample", "X", "Y", "Z", "Rx", "Ry", "Rz", "x1", "x2", "x3", "y1", "y2", "y3", "z1", "z2", "z3", "Picked correctly"]
    for item in headers:
        worksheet.write(row, column, item)
        column+=1

    rospy.init_node('test_results_picking_node')
    base_frame = "base"

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    broadcaster = tf2_ros.TransformBroadcaster()
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    rtde_c = rtde_control.RTDEControlInterface("192.168.12.245")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.12.245")

    tool_frame = "wrist_3_link"
    tf_transform_base_component = tfBuffer.lookup_transform(base_frame, "tool0", rospy.Time(), rospy.Duration(2.0))
    t, rot_vector, y_axis_original = get_t_rotvector_component(tf_transform_base_component=tf_transform_base_component)

    t = [tf_transform_base_component.transform.translation.x,
         tf_transform_base_component.transform.translation.y,
         tf_transform_base_component.transform.translation.z]

    quaternion = [tf_transform_base_component.transform.rotation.x,
                  tf_transform_base_component.transform.rotation.y,
                  tf_transform_base_component.transform.rotation.z,
                  tf_transform_base_component.transform.rotation.w]

    rot_matrix_original = R.from_quat(quaternion).as_dcm()
    rot_matrix = R.from_quat(quaternion).as_dcm()

    rot_vector_original = R.from_quat(quaternion).as_rotvec()
    rot_vector = R.from_quat(quaternion).as_rotvec()

    x_axis_original = rot_matrix_original[:3,0]
    y_axis_original = rot_matrix_original[:3,1]
    z_axis_original = rot_matrix_original[:3,2]

    y_axis = rot_matrix[:3,1]
    y_axis[2] = 0.0
    x_axis = np.array([-y_axis[1], y_axis[0], 0.0])

    rot_matrix[:3,0] = np.transpose(x_axis)
    rot_matrix[:3,2] = np.transpose(np.array([0.0, 0.0, -1.0]))
    z_axis = rot_matrix[:3,2]
    rot_vector = R.from_dcm(rot_matrix).as_rotvec()

    pose_data = []
    column = 0
    row+=1

    pose_data.append("Input Values")
    pose_data.append(t[0])
    pose_data.append(t[1])
    pose_data.append(t[2])
    pose_data.append(rot_vector_original[0])
    pose_data.append(rot_vector_original[1])
    pose_data.append(rot_vector_original[2])
    pose_data.append(x_axis_original[0])
    pose_data.append(x_axis_original[1])
    pose_data.append(x_axis_original[2])
    pose_data.append(y_axis_original[0])
    pose_data.append(y_axis_original[1])
    pose_data.append(y_axis_original[2])
    pose_data.append(z_axis_original[0])
    pose_data.append(z_axis_original[1])
    pose_data.append(z_axis_original[2])

    for item in pose_data:
        worksheet.write(row, column, item)
        column+=1

    pose_data = []
    column = 0
    row+=1

    pose_data.append("Calculated Values")
    pose_data.append(t[0])
    pose_data.append(t[1])
    pose_data.append(t[2])
    pose_data.append(rot_vector[0])
    pose_data.append(rot_vector[1])
    pose_data.append(rot_vector[2])
    pose_data.append(x_axis[0])
    pose_data.append(x_axis[1])
    pose_data.append(x_axis[2])
    pose_data.append(y_axis[0])
    pose_data.append(y_axis[1])
    pose_data.append(y_axis[2])
    pose_data.append(z_axis[0])
    pose_data.append(z_axis[1])
    pose_data.append(z_axis[2])

    for item in pose_data:
        worksheet.write(row, column, item)
        column+=1

    workbook.close()

    # Offset constant parameters
    tool_offset = 0.1245
    z_picking_offset = 0.17238

    # Pick Component
    pick_component(rtde_c, rtde_r, t, rot_vector, tool_offset, z_picking_offset)

    rtde_c.gripperDisable()
    rtde_c.stopScript()
    rtde_c.disconnect()

    quit()
    rospy.spin()