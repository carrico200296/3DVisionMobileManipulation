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
import std_msgs
from scipy.spatial.transform import Rotation as R

from functions import *

import rtde_control
import rtde_receive


if __name__ == "__main__":
 
    rospy.init_node('manipulation_node')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    broadcaster = tf2_ros.TransformBroadcaster()
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    nb_components = rospy.wait_for_message("/pose_estimation/nb_components", std_msgs.msg.String, rospy.Duration(900.0))
    rtde_c = rtde_control.RTDEControlInterface("192.168.12.245")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.12.245")

    base_frame = "base"
    print(":: Number of components detected = %d" %(int(nb_components.data)))

    list_transform_base_component = []
    for i in range(int(nb_components.data)):
        component_frame = "object_frame_" + str(i+1)
        list_transform_base_component.append(tfBuffer.lookup_transform(base_frame, component_frame, rospy.Time(0)))

    for i in range(int(nb_components.data)):
        tf_transform_base_component = list_transform_base_component[i]

        t = [tf_transform_base_component.transform.translation.x,
             tf_transform_base_component.transform.translation.y,
             tf_transform_base_component.transform.translation.z]

        quaternion = [tf_transform_base_component.transform.rotation.x,
                      tf_transform_base_component.transform.rotation.y,
                      tf_transform_base_component.transform.rotation.z,
                      tf_transform_base_component.transform.rotation.w]

        rot_matrix = R.from_quat(quaternion).as_dcm()
        rot_vector = R.from_quat(quaternion).as_rotvec()
        y_axis = rot_matrix[:3,1]
        y_axis[2] = 0.0
        x_axis = np.array([-y_axis[1], y_axis[0], 0.0])
        rot_matrix[:3,0] = np.transpose(x_axis)
        rot_matrix[:3,2] = np.transpose(np.array([0.0, 0.0, -1.0]))
        rot_vector = R.from_dcm(rot_matrix).as_rotvec()
        print("Rotation Matrix after calculation:")
        print(rot_matrix)
        print("Rotation Vector after calculation:")
        print(rot_vector)

        print(":: Pose Component %d" %(i+1))
        print("   Translation:")
        print("   x: %.4f mm" %(t[0]*1000))
        print("   y: %.4f mm" %(t[1]*1000))
        print("   z: %.4f mm" %(t[2]*1000))
        print("   Rotation Axis:")
        print("   Rx: %.4f rad" %rot_vector[0])
        print("   Ry: %.4f rad" %rot_vector[1])
        print("   Rz: %.4f rad" %rot_vector[2])

        tool_offset = 0.1245
        off_set = tool_offset + 0.08

        rtde_c.moveL([t[0], t[1], t[2] + off_set, rot_vector[0], rot_vector[1], rot_vector[2]], 0.05, 0.2)
        rtde_c.gripperDisable()
        rtde_c.gripperRelease(0)
        current_TCP = rtde_r.getActualTCPPose()
        picking_pose = current_TCP[:]
        picking_pose[2] = 0.1315
        rtde_c.moveL(picking_pose, 0.05, 0.1)
        time.sleep(0.2)
        rtde_c.gripperGrasp(0)
        time.sleep(1)
        current_TCP[2] = current_TCP[2] + off_set
        rtde_c.moveL(current_TCP, 0.2, 0.1)
        placing_point = [0.1974, 0.18591, 0.24911, 2.908, -1.188, 0.0]
        rtde_c.moveL(placing_point, 0.2, 0.3)
        time.sleep(0.3)
        rtde_c.gripperRelease(0)
        time.sleep(1.0)
        #rtde_c.moveL(current_TCP, 0.2, 0.1)

    rtde_c.gripperDisable()
    rtde_c.stopScript()
    rtde_c.disconnect()

    quit()
    rospy.spin()