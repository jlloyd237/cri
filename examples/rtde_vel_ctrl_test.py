# -*- coding: utf-8 -*-
"""Simple test script for AsyncRobot class using RTDEController.
"""

import time
import threading

import numpy as np

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import RTDEController

np.set_printoptions(precision=2, suppress=True)


def main():
    work_frame = (109.1, -487.0, 341.3, 180, 0, -90)  # base frame: x->right, y->back, z->up
    #     work_frame = (487.0, -109.1, 341.3, 180, 0, 180)    # base frame: x->front, y->right, z->up

    with AsyncRobot(SyncRobot(RTDEController(ip='192.11.72.10'))) as robot:
        # For testing in URSim simulator
        #    with AsyncRobot(SyncRobot(RTDEController(ip='127.0.0.1'))) as robot:
        # Set TCP, linear speed,  angular speed and coordinate frame
        robot.tcp = (0, 0, 89.1, 0, 0, 0)
        robot.linear_speed = 50
        robot.angular_speed = 5
        robot.coord_frame = work_frame
        controller = robot.sync_robot.controller

        # Display robot info
        print("Robot info: {}".format(robot.info))

        # Display initial joint angles
        print("Initial joint angles: {}".format(robot.joint_angles))

        # Display initial pose in work frame
        print("Initial pose in work frame: {}".format(robot.pose))

        # Move to origin of work frame
        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Single velocity move
        print("Making single velocity move, then stopping ...")
        controller.move_linear_velocity((-20, -20, -20, 0, 0, 0), 10, 9)
        controller.stop_linear_velocity(10)

        # Sequence of velocity moves
        print("Making multiple velocity moves, then stopping ...")
        controller.move_linear_velocity((40, 0, 0, 0, 0, 0), 20, 3)
        print("Calculating next move ...")
        time.sleep(2)
        controller.move_linear_velocity((0, 40, 0, 0, 0, 0), 20, 3)
        print("Calculating next move ...")
        time.sleep(2)
        controller.move_linear_velocity((0, 0, 40, 0, 0, 0), 20, 3)
        print("Calculating next move ...")
        time.sleep(2)
        controller.stop_linear_velocity(10)

        print("Waiting for 5 secs ...")
        time.sleep(5)

        print("Making multiple velocity moves, then stopping (multi-threaded) ...")

        # Sequence of velocity moves on background thread
        running = False
        lock = threading.Lock()
        velocity = (0, 0, 0, 0, 0, 0)
        accel = 10
        ret_time = 1/30

        def worker():
            while running:
                with lock:
                    worker_velocity = velocity
                    worker_accel = accel
                    worker_ret_time = ret_time
                controller.move_linear_velocity(worker_velocity, worker_accel, worker_ret_time)

        thread = threading.Thread(target=worker, args=[], kwargs={})
        running = True
        thread.start()

        try:
            with lock:
                velocity = (-40, 0, 0, 0, 0, 0)
            print("Calculating next move ...")
            time.sleep(2)
            with lock:
                velocity = (0, -40, 0, 0, 0, 0)
            print("Calculating next move ...")
            time.sleep(2)
            with lock:
                velocity = (0, 0, -40, 0, 0, 0)
            print("Calculating next move ...")
            time.sleep(2)
            with lock:
                velocity = (0, 0, 0, 0, 0, 0)
            print("Calculating next move ...")
            time.sleep(2)

        finally:
            running = False
            thread.join()

        print("Waiting for 5 secs ...")
        time.sleep(5)

        # Move to origin of work frame
        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))

        print("Final target pose in work frame: {}".format(robot.target_pose))
        print("Final pose in work frame: {}".format(robot.pose))


if __name__ == '__main__':
    main()
