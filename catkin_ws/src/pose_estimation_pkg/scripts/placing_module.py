#! /usr/bin/env python

import rtde_control
import rtde_receive

if __name__ == "__main__":

    rtde_c = rtde_control.RTDEControlInterface("192.168.12.245")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.12.245")
    
    task_frame = [0, 0, 0, 0, 0, 0]
    selection_vector = [0, 0, 1, 0, 0, 0]
    wrench_down = [0, 0, -5, 0, 0, 0]
    wrench_up = [0, 0, 5, 0, 0, 0]
    force_type = 2
    limits = [2, 2, 1.5, 1, 1, 1]

    # Move to placing_pose
    placing_pose = rtde_r.getActualTCPPose()
    rtde_c.moveL(placing_pose, 0.2, 0.3)
    time.sleep(0.3)
    # Placing strategy
    speed_z = [0, 0, -0.05, 0, 0, 0]
    rtde_c.moveUntilContact(xd = speed_z, direction=rtde_r.getTargetTCPSpeed(), acceleration=0.1)
    rtde_c.gripperRelease(0)
    time.sleep(1.0)

    '''
    # Move to placing_pose and close gripper
    rtde_c.moveL(placing_pose, 0.2, 0.3)
    time.sleep(0.3)
    rtde_c.gripperGrasp(0)
    time.sleep(1)

    # Move to adjustment_pose
    adjustment_pose = []
    rtde_c.moveL(adjustment_pose, 0.2, 0.3)
    time.sleep(0.3)
    adjustment_pose = rtde_r.getActualTCPPose()
    adjustment_pose[2] = 
    rtde_c.moveL(adjustment_pose, 0.2, 0.3)
    time.sleep(0.3)
    rtde_c.gripperRelease(0) # maybe we can use another configuration of the gripper
    time.sleep(1.0)
    initial_TCP = rtde_r.getActualTCPPose()

    speed_rz = [0, 0, 0, 0, 0, 0.05]
    while dt_z < 0.05:

        rtde_c.forceMode(task_frame, selection_vector, wrench_down, force_type, limits)
        current_TCP = rtde_r.getActualTCPPose()
        dt_z = initial_TCP - current_TCP
        rtde_c.moveUntilContact(xd=speed_rz, direction=rtde_r.getTargetTCPSpeed(), acceleration=0.1)

    rtde_c.forceModeStop()
    '''
    rtde_c.gripperDisable()
    rtde_c.stopScript()
    rtde_c.disconnect()