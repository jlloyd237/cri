# -*- coding: utf-8 -*-
"""Simple test script for AsyncRobot class using FrankxController.
"""

import time

import numpy as np

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import PyfrankaController

np.set_printoptions(precision=2, suppress=True)


def main():
    base_frame = (0, 0, 0, 0, 0, 0)
    work_frame = (400, 0, 300, 180, 0, 180)   # base frame: x->front, y->left, z->down

    with AsyncRobot(SyncRobot(PyfrankaController(ip='172.16.0.1'))) as robot:
        controller = robot.sync_robot.controller

        # Set robot axes and TCP
        robot.axes = 'sxyz'     # static/extrinsic frame xyz convention
        robot.tcp = (0, 0, 75, 0, 0, 225)

        # Set Franka-specific robot parameters
        controller.set_joint_impedance((3000, 3000, 3000, 2500, 2500, 2000, 2000))
        controller.rel_velocity = 0.1
        controller.rel_accel = 0.1
        controller.rel_jerk = 0.1

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

        print("Moving forward with constant velocity in x direction ...")
        controller.move_linear_velocity((30, 0, 0, 0, 0, 0))
        for i in range(10):
            print("Pose during move forward: {}".format(robot.pose))
            time.sleep(0.1)
        time.sleep(4)
        print("Reversing with constant velocity in -x direction...")
        controller.move_linear_velocity((-30, 0, 0, 0, 0, 0))
        for i in range(10):
            print("Pose during move backward: {}".format(robot.pose))
            time.sleep(0.1)
        time.sleep(4)
        print("Stopping ...")
        controller.move_linear_velocity((0, 0, 0, 0, 0, 0))

        print("Moving forward with constant velocity in x direction ...")
        controller.move_linear_velocity((30, 0, 0, 0, 0, 0))
        time.sleep(5)
        print("Moving forward with constant velocity in y direction ...")
        controller.move_linear_velocity((0, 30, 0, 0, 0, 0))
        time.sleep(5)
        print("Moving forward with constant velocity in z direction ...")
        controller.move_linear_velocity((0, 0, 30, 0, 0, 0))
        time.sleep(5)
        print("Stopping ...")
        controller.move_linear_velocity((0, 0, 0, 0, 0, 0))

        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))

        print("Starting circular motion in yz-plane ...")
        theta = np.linspace(0, 2 * np.pi)
        vr = 20
        for t in theta:
            vy = vr * np.cos(t)
            vz = vr * np.sin(t)
            controller.move_linear_velocity((0, vy, vz, 0, 0, 0))
            time.sleep(0.2)
        print("Stopping ...")
        controller.move_linear_velocity((0, 0, 0, 0, 0, 0))

        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))

        print("Final target pose in work frame: {}".format(robot.target_pose))
        print("Final pose in work frame: {}".format(robot.pose))


if __name__ == '__main__':
    main()