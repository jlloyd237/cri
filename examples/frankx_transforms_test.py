# -*- coding: utf-8 -*-
"""Simple test script for tcp offset transforms used in frankx controller.
"""

import warnings
from abc import ABC, abstractmethod

import numpy as np

from cri.transforms import quat2euler, euler2quat, transform, inv_transform

np.set_printoptions(precision=2, suppress=True)


def offset_tcp(tcp, offset):
    return inv_transform(offset, tcp)

def offset_pose_to_pose(offset_pose, offset):
    inv_offset = inv_transform(euler2quat((0,0,0,0,0,0)), offset)
    return inv_transform(inv_offset, offset_pose)

def pose_to_offset_pose(pose, offset):
    return inv_transform(offset, pose)

def main():
    # tcp = (0, 0, 89.1, 0, 0, -45)
    tcp = (0, 0, 89.1, 0, 0, -135)
    print(f"tcp: {tcp}")
    # offset = (0, 0, 0, -180, 0, 0)
    offset = (0, 0, 0, 180, 0, 180)
    print(f"offset: {offset}")
    os_tcp = quat2euler(offset_tcp(euler2quat(tcp), euler2quat(offset)))
    print(f"offset tcp: {os_tcp}")

    # pose = (0, 0, 0, -180, 0, 0)
    pose = (0, 0, 0, 180, 0, 180)
    print(f"pose: {pose}")
    os_pose = quat2euler(pose_to_offset_pose(euler2quat(pose), euler2quat(offset)))
    print(f"offset pose: {os_pose}")
    rec_pose = quat2euler(offset_pose_to_pose(euler2quat(os_pose), euler2quat(offset)))
    print(f"recovered pose: {rec_pose}")


if __name__ == '__main__':
    main()
