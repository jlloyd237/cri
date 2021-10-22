# -*- coding: utf-8 -*-
"""Simple test script for AsyncRobot class using RTDEController.
"""

import time

import numpy as np

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import RTDEController

np.set_printoptions(precision=2, suppress=True)


def main():
    base_frame = (0, 0, 0, 0, 0, 0)
    work_frame = (109.1, -487.0, 341.3, 180, 0, -90)   # base frame: x->right, y->back, z->up
#     work_frame = (487.0, -109.1, 341.3, 180, 0, 180)    # base frame: x->front, y->right, z->up
    
    with AsyncRobot(SyncRobot(RTDEController(ip='192.11.72.10'))) as robot:
    # For testing in URSim simulator
#    with AsyncRobot(SyncRobot(RTDEController(ip='127.0.0.1'))) as robot:
        # Set TCP, linear speed,  angular speed and coordinate frame
        robot.tcp = (0, 0, 89.1, 0, 0, 0)
        robot.linear_speed = 100
        robot.angular_speed = 10
        robot.coord_frame = work_frame

        # Display robot info
        print("Robot info: {}".format(robot.info))

        # Display initial joint angles
        print("Initial joint angles: {}".format(robot.joint_angles))

        # Display initial pose in work frame
        print("Initial pose in work frame: {}".format(robot.pose))

        # Move to origin of work frame
        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Increase and decrease all joint angles
        print("Increasing and decreasing all joint angles ...")
        robot.move_joints(robot.joint_angles + (10,)*6)
        print("Target joint angles after increase: {}".format(robot.target_joint_angles))
        print("Joint angles after increase: {}".format(robot.joint_angles))
        robot.move_joints(robot.joint_angles - (10,)*6)
        print("Target joint angles after decrease: {}".format(robot.target_joint_angles))
        print("Joint angles after decrease: {}".format(robot.joint_angles))

        # Move backward and forward
        print("Moving backward and forward ...")
        robot.move_linear((50, 0, 0, 0, 0, 0))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Move right and left
        print("Moving right and left ...")
        robot.move_linear((0, 50, 0, 0, 0, 0))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Move down and up
        print("Moving down and up ...")
        robot.move_linear((0, 0, 50, 0, 0, 0))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Roll right and left
        print("Rolling right and left ...")
        robot.move_linear((0, 0, 0, 30, 0, 0))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Roll forward and backward
        print("Rolling forward and backward ...")
        robot.move_linear((0, 0, 0, 0, 30, 0))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Turn clockwise and anticlockwise around work frame z-axis
        print("Turning clockwise and anticlockwise around work frame z-axis ...")
        robot.move_linear((0, 0, 0, 0, 0, 30))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Make a circular move down/up, via a point on the right/left
        print("Making a circular move down and up, via a point on the right/left ...")
        robot.blend_radius = 10
        robot.move_circular((0, 20, 20, 0, 0, 0), (0, 0, 40, 0, 0, 0))
        robot.blend_radius = 0
        robot.move_circular((0, -20, 20, 0, 0, 0), (0, 0, 0, 0, 0, 0))

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

        # Increase blend radius and move through a sequence of waypoints
        print("Increasing blend radius and moving through a sequence of waypoints ...")
        robot.blend_radius = 20
        print("Moving to first waypoint ...")
        robot.move_linear((100, 0, 0, 0, 0, 0))
        print("Moving to second waypoint ...")
        robot.move_linear((100, 100, 0, 0, 0, 0))
        robot.blend_radius = 0
        print("Moving to final destination ...")
        robot.move_linear((100, 100, 100, 0, 0, 0))
        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))
        print("Final target pose in work frame: {}".format(robot.target_pose))
        print("Final pose in work frame: {}".format(robot.pose))

        # Pause before commencing asynchronous tests
        print("Waiting for 5 secs ...")
        time.sleep(5)
        print("Repeating test sequence for asynchronous moves ...")

        # Increase and decrease all joint angles (async)
        print("Increasing and decreasing all joint angles ...")
        robot.async_move_joints(robot.joint_angles + (10,)*6)
        print("Getting on with something else while command completes ...")
        robot.async_result()
        print("Target joint angles after increase: {}".format(robot.target_joint_angles))
        print("Joint angles after increase: {}".format(robot.joint_angles))
        robot.async_move_joints(robot.joint_angles - (10,)*6)
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

        # Make a circular move down/up, via a point on the right/left
        print("Making a circular move down and up, via a point on the right/left (async) ...")
        robot.blend_radius = 10
        robot.async_move_circular((0, 20, 20, 0, 0, 0), (0, 0, 40, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.blend_radius = 0
        robot.async_move_circular((0, -20, 20, 0, 0, 0), (0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        # Move to offset pose then tap down and up in sensor frame (async)
        print("Moving to 50 mm / 30 deg offset in pose (async) ...")
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

        # Increase blend radius and move through a sequence of waypoints (async)
        print("Increasing blend radius and moving through a sequence of waypoints ...")
        robot.blend_radius = 20
        print("Moving to first waypoint ...")
        robot.async_move_linear((100, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        print("Moving to second waypoint ...")
        robot.async_move_linear((100, 100, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.blend_radius = 0
        print("Moving to final destination ...")
        robot.async_move_linear((100, 100, 100, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        print("Moving to origin of work frame ...")
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        print("Final target pose in work frame: {}".format(robot.target_pose))
        print("Final pose in work frame: {}".format(robot.pose))

     
if __name__ == '__main__':
    main()