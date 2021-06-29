# 3DVisionMobileManipulation
3D Vision-based Mobile Manipulation for Automated Assembly Line Feeding.

The three software modules of this project have been implemented through six ROS nodes: MIR Rest API node, ArUco ROI detection node, scene filtering node, scene reconstruction node, pose estimation node and manipulation node.

The scripts of these nodes are located at catkin_ws/src/pose_estimation_pkg/scripts/.
The real environment test, once the real hardware is set up, can be executed by running: $ roslaunch pose_estimation_pkg pose_estimation_pipeline.launch
