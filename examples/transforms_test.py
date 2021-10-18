# -*- coding: utf-8 -*-
"""Simple test script for transforms.
"""

import time

import numpy as np

from cri.transforms import euler2quat, quat2euler, euler2mat, mat2euler, transform, inv_transform

np.set_printoptions(precision=2, suppress=True)


def main():
    axes = 'rxyz'

    # test inv_transform
    x1 = (10, 20, 30, 40, 50, 60)
    x2 = (0,0,0,180,10,180)
    print("x1:", x1)
    print("x2:", x2)

    print("\nUsing homogeneous transforms:")
    x1_m = euler2mat(x1, axes)
    x2_m = euler2mat(x2, axes)
    x3_m = np.dot(np.linalg.inv(x2_m), x1_m)
    x3 = mat2euler(x3_m, axes)
    print("x1 expressed in x2 frame:", x3)
    x4_m = np.dot(x2_m, x3_m)
    x4 = mat2euler(x4_m, axes)
    print("... and transformed back to base frame:", x4)

    print("\nUsing quaternions:")
    x1_q = euler2quat(x1, axes)
    x2_q = euler2quat(x2, axes)
    x3_q = transform(x1_q, x2_q)
    x3 = quat2euler(x3_q, axes)
    print("x1 expressed in x2 frame:", x3)
    x4_q = inv_transform(x3_q, x2_q)
    x4 = quat2euler(x4_q, axes)
    print("... and transformed back to base frame:", x4)


if __name__ == '__main__':
    main()