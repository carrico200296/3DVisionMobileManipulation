#! /usr/bin/env python

import numpy as np
import rospy
import tf
import scipy
from scipy.spatial.transform import Rotation as R

# EXAMPLE: this a transfomration got it from base to object_frame_1: rosrun tf tf_echo base object_frame_1
# it is used to test if the UR can pick it up
'''
:: Manipulation Module - Pose Component 1
   x: 410.3244 mm
   y: 73.5621 mm
   z: 16.6593 mm
   qx: 0.4779 rad
   qy: -0.5162 rad
   qz: -0.5695 rad
   qw: 0.4253 rad
Roll: 1.56043 rad
Pitch: 0.10547 rad
Yaw: -1.75435 rad
[[-0.18151243 -0.00902165 -0.98334727  0.        ]
 [-0.97773759 -0.10539134  0.18144387  0.        ]
 [-0.1052732   0.9943899   0.01030904  0.        ]
 [ 0.          0.          0.          1.        ]]
'''
T_matrix = [[-0.18151243, -0.00902165, -0.98334727,  0.0],
            [-0.97773759, -0.10539134,  0.18144387,  0.0],
            [-0.1052732 ,  0.9943899 ,  0.01030904,  0.0],
            [ 0.0       ,  0.0      ,  0.0       ,  1.0]]
rot_mtx = R.from_quat([0.4779, -0.5162, -0.5695, 0.4253])
#print(rot_mtx.as_dcm())
#print(rot_mtx.as_rotvec())
#print(rot_mtx.as_euler('zyx', degrees=False))

ang = np.pi/2 # in rad
R_matrix = rot_mtx.as_dcm()
rot_x = np.array([[1.0, 0.0, 0.0],
                  [0.0, np.cos(ang), -np.sin(ang)],
                  [0.0, np.sin(ang), np.cos(ang)]])
rot_x = R.from_euler('x', 90, degrees=True).as_dcm()
R_matrix_transform = R.from_dcm(np.linalg.multi_dot([R_matrix, rot_x]))
print(R_matrix_transform.as_rotvec())


# TF TRANSFORMATIONS for Realsense D415 and URe robot fixed station Robot Lab
# info: RPY (roll pitch yaw) is rotation around ZYX

'''
$ rosrun tf tf_echo camera_link camera_color_optical_frame
- Translation: [0.000, 0.015, 0.000]
- Rotation: in Quaternion [-0.498, 0.499, -0.501, 0.501]
            in RPY (radian) [-1.565, 0.001, -1.572]
            in RPY (degree) [-89.688, 0.061, -90.072]
'''
T_cameraLink_colorOptical = np.eye(4)
trans_cameraLink_colorOptical = [0.000, 0.015, 0.000]
R_cameraLink_colorOptical = [ [-0.0012037,  0.0057975,  0.9999825],
                              [-0.9999987,  0.0009930, -0.0012094],
                              [-0.0010000, -0.9999827,  0.0057963] ]
R_cameraLink_colorOptical = np.asarray(R_cameraLink_colorOptical)
T_cameraLink_colorOptical[:3,:3] = R_cameraLink_colorOptical 
T_cameraLink_colorOptical[:3,3] = trans_cameraLink_colorOptical 

'''
$ rosrun tf tf_echo camera_color_optical_frame camera_link
- Translation: [0.015, 0.000, -0.000]
- Rotation: in Quaternion [0.498, -0.499, 0.501, 0.501]
            in RPY (radian) [-0.227, -1.565, 1.797]
            in RPY (degree) [-13.030, -89.680, 102.970]
'''
T_colorOptical_cameraLink = np.eye(4)
trans_colorOptical_cameraLink = [0.015, 0.000, -0.000]
R_colorOptical_cameraLink = [ [-0.0013000, -0.9999988, -0.0008000],
                               [0.0056486,  0.0007926, -0.9999837],
                               [0.9999832, -0.0013045,  0.0056476] ]
T_colorOptical_cameraLink[:3,:3] = R_colorOptical_cameraLink
T_colorOptical_cameraLink[:3,3] = trans_colorOptical_cameraLink
#R_colorOptical_cameraLink = np.linalg.inv(R_cameraLink_colorOptical) same approach


'''
$ rosrun tf tf_echo wrist_3_link camera_color_optical_frame
- Translation: [-0.034, -0.084, 0.019]
- Rotation: in Quaternion [0.007, -0.002, -0.007, 1.000]
            in RPY (radian) [0.014, -0.003, -0.015]
            in RPY (degree) [0.796, -0.173, -0.858]
from calibration: xyz="-0.0344852 -0.0843553 0.0191218" rpy="0.0138443 -0.00323099 -0.0149272"
'''
T_wrist_colorOptical = np.eye(4)
trans_wrist_colorOptical = [-0.034, -0.084, 0.019]
R_wrist_colorOptical = [  [0.9998834,  0.0148805, -0.0034370],
                          [-0.0149266,  0.9997934, -0.0137941],
                          [0.0032310,  0.0138438,  0.9998990] ]
R_wrist_colorOptical = np.asarray(R_wrist_colorOptical)
T_wrist_colorOptical[:3,:3] = R_wrist_colorOptical
T_wrist_colorOptical[:3,3] = trans_wrist_colorOptical

# Transformation of /camera_link frame respect to the /wrist_3_link frame
# T_wrist_cameraLink = T_wrist_colorOptical x T_colorOptical_cameraLink
T_wrist_cameraLink = np.linalg.multi_dot([T_wrist_colorOptical, T_colorOptical_cameraLink])
trans_wrist_cameraLink = T_wrist_cameraLink[:3,3]
q_wrist_cameraLink = tf.transformations.quaternion_from_matrix(T_wrist_cameraLink)

print("Homogeneous Transformation of /camera_link respect to /wrist_3_frame:")
print(T_wrist_cameraLink)
print("Translation: X Y Z")
print(trans_wrist_cameraLink)
print("Rotation: quaternion")
print(q_wrist_cameraLink)