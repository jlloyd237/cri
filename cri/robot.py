# -*- coding: utf-8 -*-
"""Robot class provide a common, higher-level interface to various robot arms.
"""

import queue
from threading import Thread

import numpy as np

from cri.transforms import euler2quat, quat2euler, transform, inv_transform


class InvalidEulerAxes(ValueError):
    pass


class InvalidJointAngles(ValueError):
    pass


class InvalidEulerPose(ValueError):
    pass


class AsyncBusy(RuntimeError):
    pass


class AsyncNotBusy(RuntimeError):
    pass    


def check_joint_angles(joint_angles):
    if len(joint_angles) != 6:
        raise InvalidJointAngles


def check_pose(pose):
    if len(pose) != 6:
        raise InvalidEulerPose
            

class SyncRobot:
    """Synchronous robot class provides synchronous (blocking) movement
    primitives.
    
    Poses and coordinate frames are specified using 3D Euclidean positions
    and Euler rotations.  This makes it easier to specify poses and coordinate
    frames using a more intuitive representation than quaternions. 
    """
    
    EULER_AXES = ('sxyz', 'sxyx', 'sxzy', 'sxzx', 'syzx', 'syzy',
                  'syxz', 'syxy', 'szxy', 'szxz', 'szyx', 'szyz',
                  'rzyx', 'rxyx', 'ryzx', 'rxzx', 'rxzy', 'ryzy',
                  'rzxy', 'ryxy', 'ryxz', 'rzxz', 'rxyz', 'rzyz',
                  )
    
    def __init__(self, ctrl):
        self._ctrl = ctrl
        try:
            self.axes = 'rxyz'
            self.tcp = (0, 0, 0, 0, 0, 0)           # tool flange frame (euler)
            self.coord_frame = (0, 0, 0, 0, 0, 0)   # base frame (euler)
            self.linear_speed = 20                  # mm/s
            self.angular_speed = 20                 # deg/s
            self.blend_radius = 0                   # mm
        except:
            self._ctrl.close()
            raise

    def __repr__(self):      
        return "{} ({})".format(self.__class__.__name__, self.info)

    def __str__(self):
        return self.__repr__()

    def __enter__(self):
        return self
        
    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    @property
    def info(self):
        """Returns a unique robot identifier string.
        """
        return self._ctrl.info

    @property
    def axes(self):
        """Returns the Euler axes used to specify frames and poses.

        Allowable values:
        'sxyz', 'sxyx', 'sxzy', 'sxzx', 'syzx', 'syzy',
        'syxz', 'syxy', 'szxy', 'szxz', 'szyx', 'szyz',
        'rzyx', 'rxyx', 'ryzx', 'rxzx', 'rxzy', 'ryzy',
        'rzxy', 'ryxy', 'ryxz', 'rzxz', 'rxyz', 'rzyz'
        
        'r' = rotating/intrinsic/local frame
        's' = static/extrinsic/global frame
        """
        return self._axes
    
    @axes.setter
    def axes(self, axes):
        if axes not in self.EULER_AXES:
            raise InvalidEulerAxes
        self._axes = axes

    @property
    def tcp(self):
        """Returns the tool center point (TCP) of the robot.
        
        The TCP is specified in the output flange frame, which is located at
        the intersection of the tool flange center axis and the flange face,
        with the z-axis aligned with the tool flange center axis.
        
        tool = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Euclidean position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        """
        return quat2euler(self._ctrl.tcp, self._axes)

    @tcp.setter
    def tcp(self, tcp):
        """Sets the tool center point (TCP) of the robot.
        
        The TCP is specified in the output flange frame, which is located at
        the intersection of the tool flange center axis and the flange face,
        with the z-axis aligned with the tool flange center axis.
        
        tcp = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Euclidean position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        """
        check_pose(tcp)
        self._ctrl.tcp = euler2quat(tcp, self._axes)

    @property
    def coord_frame(self):
        """Returns the reference coordinate frame for the robot.
        
        frame = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Euclidean position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        """
        return quat2euler(self._coord_frame_q, self._axes)

    @coord_frame.setter
    def coord_frame(self, frame):
        """Sets the reference coordinate frame for the robot.
        
        frame = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Euclidean position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        """
        check_pose(frame)
        self._coord_frame_q = euler2quat(frame, self._axes)
        self._is_base_frame = np.array_equal(frame, (0, 0, 0, 0, 0, 0))

    @property
    def linear_speed(self):
        """Returns the linear speed of the robot TCP (mm/s).
        """
        return self._ctrl.linear_speed

    @linear_speed.setter
    def linear_speed(self, speed):
        """Sets the linear speed of the robot TCP (mm/s).
        """
        self._ctrl.linear_speed = speed

    @property
    def angular_speed(self):
        """Returns the angular speed of the robot TCP (deg/s).
        """
        return self._ctrl.angular_speed

    @angular_speed.setter
    def angular_speed(self, speed):
        """Sets the angular speed of the robot TCP (deg/s).
        """
        self._ctrl.angular_speed = speed

    @property
    def blend_radius(self):
        """Returns the robot blend radius (mm).
        """
        return self._ctrl.blend_radius

    @blend_radius.setter
    def blend_radius(self, blend_radius):
        """Sets the robot blend radius (mm).
        """
        self._ctrl.blend_radius = blend_radius

    @property
    def joint_angles(self):
        """ Returns the robot joint angles.
        
        joint_angles = (j0, j1, j2, j3, j4, j5)
        j0, j1, j2, j3, j4, j5 specify the joint angles (degrees), numbered
        from base to end effector
        """
        return self._ctrl.joint_angles

    @property
    def pose(self):
        """Returns the TCP pose in the reference coordinate frame.
        
        pose = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Euclidean position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        """
        pose_q = self._ctrl.pose
        if self._is_base_frame:
            return quat2euler(pose_q, self._axes)
        else:
            return quat2euler(transform(pose_q, self._coord_frame_q), self._axes)

    def move_joints(self, joint_angles):
        """Executes an immediate move to the specified joint angles.
        
        joint_angles = (j0, j1, j2, j3, j4, j5)
        j0, j1, j2, j3, j4, j5 specify the joint angles (degrees), numbered
        from base to end effector
        """
        check_joint_angles(joint_angles)
        self._ctrl.move_joints(joint_angles)
    
    def move_linear(self, pose):
        """Executes a linear/cartesian move from the current TCP pose to the
        specified pose in the reference coordinate frame.
        
        pose = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Euclidean position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        """
        check_pose(pose)
        pose_q = euler2quat(pose, self._axes)
        if self._is_base_frame:
            self._ctrl.move_linear(pose_q)
        else:
            self._ctrl.move_linear(inv_transform(pose_q, self._coord_frame_q))

    def move_circular(self, via_pose, end_pose):
        """Executes a movement in a circular path from the current TCP pose,
        through via_pose, to end_pose in the reference coordinate frame.

        via_pose, end_pose = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Euclidean position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        """
        check_pose(via_pose)
        check_pose(end_pose)
        via_pose_q = euler2quat(via_pose, self._axes)
        end_pose_q = euler2quat(end_pose, self._axes)
        if self._is_base_frame:
            self._ctrl.move_circular(via_pose_q, end_pose_q)
        else:
            self._ctrl.move_circular(inv_transform(via_pose_q, self._coord_frame_q),
                                     inv_transform(end_pose_q, self._coord_frame_q))
   
    def close(self):
        """Releases any resources held by the robot (e.g., sockets).
        """
        self._ctrl.close()      


class AsyncRobot:
    """Asynchronous robot class provides synchronous (blocking) and
    asynchronous (non-blocking) movement primitives.

    Poses and coordinate frames are specified using a 3D Euclidean position
    and a 3D Euler rotation.  This makes it easier to specify poses and
    coordinate frames using a more intuitive representation than quaternions.    
    """
    
    def __init__(self, *args, **kwargs):
        self._robot = SyncRobot(*args, **kwargs)
        try:
            self._worker = None
            self._results = queue.Queue()
            self._busy = False
        except:
            self._robot.close()
            raise

    def __repr__(self):      
        return "{} ({})".format(self.__class__.__name__, self.info)

    def __str__(self):
        return self.__repr__()

    def __enter__(self):
        return self
        
    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def __getattr__(self, name):
        """Delegates any undefined methods, properties or attributes to robot
        object after checking to see whether it is busy.
        """
        if self._busy:
            raise AsyncBusy        
        return getattr(self._robot, name)
    
    def __setattr__(self, name, value):
        """Delegates setting of named properties to robot object after checking
        to see whether it is busy.
        """
        if name in ('tcp', 'coord_frame', 'linear_speed', 'angular_speed', 'blend_radius'):
            if self._busy:
                raise AsyncBusy
            setattr(self._robot, name, value)
        else:
            super().__setattr__(name, value)

    def async_move_joints(self, joint_angles):
        """Executes an immediate move to the specified joint angles.
        
        joint_angles = (j0, j1, j2, j3, j4, j5)
        j0, j1, j2, j3, j4, j5 specify the joint angles (degrees), numbered
        from base to end effector
        """
        if self._busy:
            raise AsyncBusy  
        self._busy = True
        self._worker = Thread(target=lambda joint_angles, results: \
                              results.put(self._robot.move_joints(joint_angles)), \
                              args=(joint_angles, self._results))
        self._worker.start()
    
    def async_move_linear(self, pose):
        """Executes a linear/cartesian move from the current TCP pose to the
        specified pose in the reference coordinate frame.
        
        pose = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Euclidean position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        """
        if self._busy:
            raise AsyncBusy   
        self._busy = True
        self._worker = Thread(target=lambda pose, results: \
                              results.put(self._robot.move_linear(pose)), \
                              args=(pose, self._results))
        self._worker.start()

    def async_move_circular(self, via_pose, end_pose):
        """Executes a movement in a circular path from the current TCP pose,
        through via_pose, to end_pose in the reference coordinate frame.

        via_pose, end_pose = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Euclidean position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        """
        if self._busy:
            raise AsyncBusy
        self._busy = True
        self._worker = Thread(target=lambda via_pose, end_pose, results: \
                              results.put(self._robot.move_circular(via_pose, end_pose)),\
                              args=(via_pose, end_pose, self._results))
        self._worker.start()
    
    def async_result(self):
        """Blocks for result of asynchronous operation.
        """
        if not self._busy:
            raise AsyncNotBusy            
        self._worker.join()
        result = self._results.get()
        self._busy = False
        return result
    
    def async_done(self):
        """Checks to see whether asynchronous operation has finished.
        """
        if not self._busy:
            raise AsyncNotBusy  
        return not self._worker.is_alive()
        
    def close(self):
        """Releases any resources held by the robot (e.g., sockets).
        """
        self._robot.close()
