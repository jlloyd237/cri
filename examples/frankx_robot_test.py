# -*- coding: utf-8 -*-
"""Simple test script for AsyncRobot class using FrankxController.
"""

import time

import numpy as np

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import FrankxController

np.set_printoptions(precision=2, suppress=True)


def main():
    base_frame = (0, 0, 0, 0, 0, 0)
    work_frame = (400, 0, 300, 180, 0, 180)   # base frame: x->front, y->right, z->up

    with AsyncRobot(SyncRobot(FrankxController(ip='172.16.0.1'))) as robot:
        # Set robot axes and TCP
        robot.axes = 'sxyz'     # static/extrinsic frame xyz convention
        robot.tcp = (0, 0, 75, 0, 0, -135)

        # Set Franka-specific robot parameters
        robot.sync_robot.controller.rel_velocity = 0.3
        robot.sync_robot.controller.rel_accel = 0.2
        robot.sync_robot.controller.rel_jerk = 0.1

        # Display robot info
        print("Robot info: {}".format(robot.info))

        # Initialize all 7 joints and display joint angles and pose
        robot.move_joints((0, -35, 0, -150, 0, 115, -45))
        print("Initial joint angles: {}".format(robot.joint_angles))
        print("Initial pose in base frame: {}".format(robot.pose))
        robot.coord_frame = work_frame
        print("Initial pose in work frame: {}".format(robot.pose))

        # Move to origin of work frame
        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))
        print("Pose at origin of work frame: {}".format(robot.pose))

        # Increase and decrease all joint angles
        print("Increasing and decreasing all joint angles ...")
        robot.move_joints(robot.joint_angles + (10,)*7)
        print("Target joint angles after increase: {}".format(robot.target_joint_angles))
        print("Joint angles after increase: {}".format(robot.joint_angles))
        robot.move_joints(robot.joint_angles - (10,)*7)
        print("Target joint angles after decrease: {}".format(robot.target_joint_angles))
        print("Joint angles after decrease: {}".format(robot.joint_angles))

        # Move backward and forward
        print("Moving backward and forward ...")
        robot.move_linear((50, 0, 0, 0, 0, 0))
        print("Pose in work frame: {}".format(robot.pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Move right and left
        print("Moving right and left ...")
        robot.move_linear((0, 50, 0, 0, 0, 0))
        print("Pose in work frame: {}".format(robot.pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Move down and up
        print("Moving down and up ...")
        robot.move_linear((0, 0, 50, 0, 0, 0))
        print("Pose in work frame: {}".format(robot.pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Roll right and left
        print("Rolling right and left ...")
        robot.move_linear((0, 0, 0, 30, 0, 0))
        print("Pose in work frame: {}".format(robot.pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Roll forward and backward
        print("Rolling forward and backward ...")
        robot.move_linear((0, 0, 0, 0, 30, 0))
        print("Pose in work frame: {}".format(robot.pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Turn clockwise and anticlockwise around work frame z-axis
        print("Turning clockwise and anticlockwise around work frame z-axis ...")
        robot.move_linear((0, 0, 0, 0, 0, 30))
        print("Pose in work frame: {}".format(robot.pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Move to offset pose then tap down and up in sensor frame
        print("Moving to 50 mm / 30 deg offset in pose ...")
        robot.move_linear((50, 50, 50, 30, 30, 30))
        print("Target pose after offset move: {}".format(robot.target_pose))
        print("Pose after offset move: {}".format(robot.pose))
        print("Tapping down and up ...")
        robot.coord_frame = base_frame
        robot.coord_frame = robot.target_pose
        robot.move_linear((0, 0, 50, 0, 0, 0))
        robot.move_linear((0, 0, 0, 0, 0, 0))
        robot.coord_frame = work_frame
        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))
        print("Pose at origin of work frame: {}".format(robot.pose))

        # Pause before commencing asynchronous tests
        print("Waiting for 5 secs ...")
        time.sleep(5)
        print("Repeating test sequence for asynchronous moves ...")

        # Increase and decrease all joint angles (async)
        print("Increasing and decreasing all joint angles ...")
        robot.async_move_joints(robot.joint_angles + (10,)*7)
        print("Getting on with something else while command completes ...")
        robot.async_result()
        print("Target joint angles after increase: {}".format(robot.target_joint_angles))
        print("Joint angles after increase: {}".format(robot.joint_angles))
        robot.async_move_joints(robot.joint_angles - (10,)*7)
        print("Getting on with something else while command completes ...")
        robot.async_result()
        print("Target joint angles after decrease: {}".format(robot.target_joint_angles))
        print("Joint angles after decrease: {}".format(robot.joint_angles))

        # Move backward and forward (async)
        print("Moving backward and forward (async) ...")
        robot.async_move_linear((50, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        # Move right and left
        print("Moving right and left (async) ...")
        robot.async_move_linear((0, 50, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        # Move down and up (async)
        print("Moving down and up (async) ...")
        robot.async_move_linear((0, 0, 50, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        # Roll right and left (async)
        print("Rolling right and left (async) ...")
        robot.async_move_linear((0, 0, 0, 30, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        # Roll forward and backward (async)
        print("Rolling forward and backward (async) ...")
        robot.async_move_linear((0, 0, 0, 0, 30, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        # Turn clockwise and anticlockwise around work frame z-axis (async)
        print("Turning clockwise and anticlockwise around work frame z-axis (async) ...")
        robot.async_move_linear((0, 0, 0, 0, 0, 30))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        # Move to offset pose then tap down and up in sensor frame (async)
        print("Moving to 50 mm / 30 deg offset in all pose dimensions (async) ...")
        robot.async_move_linear((50, 50, 50, 30, 30, 30))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        print("Target pose after offset move: {}".format(robot.target_pose))
        print("Pose after offset move: {}".format(robot.pose))
        print("Tapping down and up (async) ...")
        robot.coord_frame = base_frame
        robot.coord_frame = robot.target_pose
        robot.async_move_linear((0, 0, 50, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.coord_frame = work_frame
        print("Moving to origin of work frame ...")
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        print("Final target pose in work frame: {}".format(robot.target_pose))
        print("Final pose in work frame: {}".format(robot.pose))


if __name__ == '__main__':
    main()