# -*- coding: utf-8 -*-
"""Functions for converting between euler- and quaternion-based
poses and coordinate frames, and for transforming between frames.
"""

import numpy as np
import transforms3d as t3d
from transforms3d.quaternions import qinverse, qmult, rotate_vector


def euler2quat(pose_e, axes='sxyz'):
    """Converts an euler rotation pose to a quaternion rotation pose.
    """
    pose_e = np.array(pose_e, dtype=np.float64).ravel()
    assert pose_e.size == 6, "Invalid euler pose"
    rot_e_rad = np.deg2rad(pose_e[3:])
    rot_q = t3d.euler.euler2quat(*rot_e_rad, axes)
    pose_q = np.concatenate((pose_e[:3], rot_q))
    return pose_q

def quat2euler(pose_q, axes='sxyz'):
    """Converts a quaternion rotation pose to an euler rotation pose.
    """
    pose_q = np.array(pose_q, dtype=np.float64).ravel()
    assert pose_q.size == 7, "Invalid quaternion pose"
    rot_e_rad = t3d.euler.quat2euler(pose_q[3:], axes)
    rot_e_deg = np.rad2deg(rot_e_rad)
    pose_e = np.concatenate((pose_q[:3], rot_e_deg))
    return pose_e

def axangle2quat(pose_a):
    """Converts an axis-angle rotation pose to a quaternion rotation pose.
    """
    pose_a = np.array(pose_a, dtype=np.float64).ravel()
    assert pose_a.size == 6, "Invalid axis-angle pose"
    rot_a_angle = np.linalg.norm(pose_a[3:])
    rot_a_axis = pose_a[3:]
    rot_q = t3d.quaternions.axangle2quat(rot_a_axis, rot_a_angle)
    pose_q = np.concatenate((pose_a[:3], rot_q))
    return pose_q

def quat2axangle(pose_q):
    """Converts a quaternion rotation pose to an axis-angle rotation pose.
    """
    pose_q = np.array(pose_q, dtype=np.float64).ravel()
    assert pose_q.size == 7, "Invalid quaternion pose"
    rot_a_axis, rot_a_angle = t3d.quaternions.quat2axangle(pose_q[3:])
    rot_a = rot_a_angle * rot_a_axis
    pose_a = np.concatenate((pose_q[:3], rot_a))    
    return pose_a

def mat2quat(pose_m):
    """Converts a 4x4 homogeneous matrix pose to a quaternion rotation pose.
    """
    pose_m = np.array(pose_m, dtype=np.float64)
    assert pose_m.shape == (4, 4), "Invalid 4x4 homogeneous matrix pose"
    pos_m = pose_m[:3, 3]
    rot_m = pose_m[:3, :3]
    rot_q = t3d.quaternions.mat2quat(rot_m)
    pose_q = np.concatenate((pos_m, rot_q))
    return pose_q

def quat2mat(pose_q):
    """Converts a quaternion rotation pose to a 4x4 homogeneous matrix.
    """
    pose_q = np.array(pose_q, dtype=np.float64).ravel()
    assert pose_q.size == 7, "Invalid quaternion pose"
    rot_m = t3d.quaternions.quat2mat(pose_q[3:])
    pos_m = pose_q[:3].reshape(3,1)
    pose_m = np.concatenate((rot_m, pos_m), axis=1)
    pose_m = np.concatenate((pose_m, np.array((0,0,0,1)).reshape(1,4)))
    return pose_m

def euler2mat(pose_e, axes='sxyz'):
    """Converts an euler rotation pose to a 4x4 homogeneous matrix pose.
    """
    pose_e = np.array(pose_e, dtype=np.float64).ravel()
    assert pose_e.size == 6, "Invalid euler pose"
    rot_e_rad = np.deg2rad(pose_e[3:])
    rot_m = t3d.euler.euler2mat(*rot_e_rad, axes)   
    pos_m = pose_e[:3].reshape(3,1)
    pose_m = np.concatenate((rot_m, pos_m), axis=1)
    pose_m = np.concatenate((pose_m, np.array((0,0,0,1)).reshape(1,4)))
    return pose_m

def mat2euler(pose_m, axes='sxyz'):
    """Converts a 4x4 homogeneous matrix pose to an euler rotation pose.
    """
    pose_m = np.array(pose_m, dtype=np.float64)
    assert pose_m.shape == (4, 4), "Invalid 4x4 homogeneous matrix pose"
    pos_m = pose_m[:3, 3]
    rot_m = pose_m[:3, :3]
    rot_e_rad = t3d.euler.mat2euler(rot_m, axes)
    rot_e_deg = np.rad2deg(rot_e_rad)
    pose_e = np.concatenate((pos_m, rot_e_deg))
    return pose_e

def transform(pose_a, frame_b_a):
    """Transforms a quaternion pose between reference frames.
        
    Transforms a pose in reference frame A to a pose in reference frame
    B (B is expressed relative to reference frame A).
    """  
    pos_b = rotate_vector(pose_a[:3] - frame_b_a[:3], qinverse(frame_b_a[3:]))
    rot_b = qmult(qinverse(frame_b_a[3:]), pose_a[3:])
    pose_b = np.concatenate((pos_b, rot_b))
    return pose_b

def inv_transform(pose_b, frame_b_a):
    """Inverse transforms  a quaternion pose between reference frames.
        
    Transforms a pose in reference frame B to a pose in reference frame
    A (B is expressed relative to A).
    """
    pos_a = rotate_vector(pose_b[:3], frame_b_a[3:]) + frame_b_a[:3]
    rot_a = qmult(frame_b_a[3:], pose_b[3:]);
    pose_a = np.concatenate((pos_a, rot_a))
    return pose_a
