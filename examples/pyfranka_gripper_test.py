# -*- coding: utf-8 -*-
"""Simple test script for AsyncRobot class using FrankxController.
"""

import time

import numpy as np

from cri.robot import SyncRobot
from cri.controller import PyfrankaController

np.set_printoptions(precision=2, suppress=True)


def main():
    base_frame = (0, 0, 0, 0, 0, 0)
    work_frame = (400, 0, 300, 180, 0, 180)   # base frame: x->front, y->left, z->down
    grasp_width = 0.01  # m
    grasp_speed = 0.01  # m/s
    grasp_force = 1.0   # N

    with SyncRobot(PyfrankaController(ip='172.16.0.1')) as robot:
        # Set robot axes and TCP
        robot.axes = 'sxyz'     # static/extrinsic frame xyz convention
        robot.tcp = (0, 0, 95, 0, 0, 225)

        # Set Franka-specific robot parameters
        robot.controller.set_joint_impedance((3000, 3000, 3000, 2500, 2500, 2000, 2000))
        robot.controller.rel_velocity = 0.1
        robot.controller.rel_accel = 0.1
        robot.controller.rel_jerk = 0.1

        # Get gripper, perform homing and check some parameters
        gripper = robot.controller.gripper
        gripper.homing()
        print("Gripper temperature: {} C".format(gripper.temperature))
        print("Gripper max width: {:.3f} m".format(gripper.max_width))
        print("Gripper width: {:.3f} m".format(gripper.width))

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
        gripper.grasp(grasp_width, grasp_speed, grasp_force)
        robot.move_joints(robot.joint_angles - (10,)*7)
        print("Target joint angles after decrease: {}".format(robot.target_joint_angles))
        print("Joint angles after decrease: {}".format(robot.joint_angles))
        gripper.homing()

        # Move down and up
        print("Moving down and up ...")
        robot.move_linear((0, 0, 50, 0, 0, 0))
        print("Pose in work frame: {}".format(robot.pose))
        gripper.grasp(grasp_width, grasp_speed, grasp_force)
        robot.move_linear((0, 0, 0, 0, 0, 0))
        gripper.homing()

        # Roll right and left
        print("Rolling right and left ...")
        robot.move_linear((0, 0, 0, 30, 0, 0))
        print("Pose in work frame: {}".format(robot.pose))
        gripper.grasp(grasp_width, grasp_speed, grasp_force)
        robot.move_linear((0, 0, 0, 0, 0, 0))
        gripper.homing()

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
        gripper.grasp(grasp_width, grasp_speed, grasp_force)
        robot.coord_frame = work_frame
        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))
        print("Pose at origin of work frame: {}".format(robot.pose))
        gripper.homing()

        print("Final target pose in work frame: {}".format(robot.target_pose))
        print("Final pose in work frame: {}".format(robot.pose))


if __name__ == '__main__':
    main()