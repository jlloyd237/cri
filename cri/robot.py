# -*- coding: utf-8 -*-
"""Robot class provide a common, higher-level interface to various robot arms.
"""

import queue
from abc import ABC, abstractmethod
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
    if not (6 <= len(joint_angles) <= 7):
        raise InvalidJointAngles


def check_pose(pose):
    if len(pose) != 6:
        raise InvalidEulerPose


class Robot(ABC):
    """Robot class provides common robot interface.

    Poses and coordinate frames are specified using 3D Cartesian positions
    and Euler rotations.  This makes it easier to specify poses and coordinate
    frames using a more intuitive representation than quaternions.
    """

    EULER_AXES = ('sxyz', 'sxyx', 'sxzy', 'sxzx', 'syzx', 'syzy',
                  'syxz', 'syxy', 'szxy', 'szxz', 'szyx', 'szyz',
                  'rzyx', 'rxyx', 'ryzx', 'rxzx', 'rxzy', 'ryzy',
                  'rzxy', 'ryxy', 'ryxz', 'rzxz', 'rxyz', 'rzyz',
                  )

    def __repr__(self):
        return "{} ({})".format(self.__class__.__name__, self.info)

    def __str__(self):
        return self.__repr__()

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    @property
    @abstractmethod
    def info(self):
        """Returns a unique robot identifier string.
        """
        pass
    
    @property
    @abstractmethod
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
        pass

    @axes.setter
    @abstractmethod
    def axes(self, axes):
        pass

    @property
    @abstractmethod
    def tcp(self):
        """Returns the tool center point (TCP) of the robot.

        The TCP is specified in the output flange frame, which is located at
        the intersection of the tool flange center axis and the flange face,
        with the z-axis aligned with the tool flange center axis.

        tool = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Cartesian position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        """
        pass

    @tcp.setter
    @abstractmethod
    def tcp(self, tcp):
        """Sets the tool center point (TCP) of the robot.

        The TCP is specified in the output flange frame, which is located at
        the intersection of the tool flange center axis and the flange face,
        with the z-axis aligned with the tool flange center axis.

        tcp = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Cartesian position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        """
        pass

    @property
    @abstractmethod
    def coord_frame(self):
        """Returns the reference coordinate frame for the robot.

        frame = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Cartesian position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        """
        pass

    @coord_frame.setter
    @abstractmethod
    def coord_frame(self, frame):
        """Sets the reference coordinate frame for the robot.

        frame = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Cartesian position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        """
        pass

    @property
    @abstractmethod
    def linear_speed(self):
        """Returns the linear speed of the robot TCP (mm/s).
        """
        pass

    @linear_speed.setter
    @abstractmethod
    def linear_speed(self, speed):
        """Sets the linear speed of the robot TCP (mm/s).
        """
        pass

    @property
    @abstractmethod
    def angular_speed(self):
        """Returns the angular speed of the robot TCP (deg/s).
        """
        pass

    @angular_speed.setter
    @abstractmethod
    def angular_speed(self, speed):
        """Sets the angular speed of the robot TCP (deg/s).
        """
        pass

    @property
    @abstractmethod
    def blend_radius(self):
        """Returns the robot blend radius (mm).
        """
        pass

    @blend_radius.setter
    @abstractmethod
    def blend_radius(self, blend_radius):
        """Sets the robot blend radius (mm).
        """
        pass

    @property
    @abstractmethod
    def joint_angles(self):
        """ Returns the robot joint angles.

        joint_angles = (j0, j1, j2, j3, j4, j5)
        j0, j1, j2, j3, j4, j5 specify the joint angles (degrees), numbered
        from base to end effector
        """
        pass

    @property
    @abstractmethod
    def target_joint_angles(self):
        """ Returns the target robot joint angles.

        joint_angles = (j0, j1, j2, j3, j4, j5)
        j0, j1, j2, j3, j4, j5 specify the joint angles (degrees), numbered
        from base to end effector
        """
        pass

    @property
    @abstractmethod
    def pose(self):
        """Returns the TCP pose in the reference coordinate frame.

        pose = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Cartesian position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        """
        pass

    @property
    @abstractmethod
    def target_pose(self):
        """Returns the target TCP pose in the reference coordinate frame.

        pose = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Cartesian position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        """
        pass

    @property
    @abstractmethod
    def elbow(self):
        """Returns the current elbow angle (degrees).
        """
        pass

    @property
    @abstractmethod
    def target_elbow(self):
        """Returns the target elbow angle (degrees).
        """
        pass

    @abstractmethod
    def move_joints(self, joint_angles):
        """Executes an immediate move to the specified joint angles.

        joint_angles = (j0, j1, j2, j3, j4, j5)
        j0, j1, j2, j3, j4, j5 specify the joint angles (degrees), numbered
        from base to end effector
        """
        pass

    @abstractmethod
    def move_linear(self, pose, elbow):
        """Executes a linear/cartesian move from the current TCP pose to the
        specified pose in the reference coordinate frame.

        pose = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Cartesian position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        elbow = target elbow angle for 7-DOF robot arm (optional; degrees)
        """
        pass

    @abstractmethod
    def move_circular(self, via_pose, end_pose, elbow):
        """Executes a movement in a circular path from the current TCP pose,
        through via_pose, to end_pose in the reference coordinate frame.

        via_pose, end_pose = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Cartesian position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        elbow = target elbow angle for 7-DOF robot arm (optional; degrees)
        """
        pass

    @abstractmethod
    def close(self):
        """Releases any resources held by the robot (e.g., sockets).
        """
        pass


class SyncRobot(Robot):
    """Synchronous robot class provides synchronous (blocking) movement
    primitives.
    """
    
    def __init__(self, controller):
        self.controller = controller
        try:
            self.axes = 'rxyz'
            self.tcp = (0, 0, 0, 0, 0, 0)           # tool flange frame (euler)
            self.coord_frame = (0, 0, 0, 0, 0, 0)   # base frame (euler)
        except:
            self.controller.close()
            raise

    @property
    def info(self):
        """Returns a unique robot identifier string.
        """
        return self.controller.info

    @property
    def axes(self):
        """Returns the Euler axes used to specify frames and poses.
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
        """
        return quat2euler(self.controller.tcp, self._axes)

    @tcp.setter
    def tcp(self, tcp):
        """Sets the tool center point (TCP) of the robot.
        """
        check_pose(tcp)
        self.controller.tcp = euler2quat(tcp, self._axes)

    @property
    def coord_frame(self):
        """Returns the reference coordinate frame for the robot.
        """
        return quat2euler(self._coord_frame_q, self._axes)

    @coord_frame.setter
    def coord_frame(self, frame):
        """Sets the reference coordinate frame for the robot.
        """
        check_pose(frame)
        self._coord_frame_q = euler2quat(frame, self._axes)

    @property
    def linear_speed(self):
        """Returns the linear speed of the robot TCP (mm/s).
        """
        return self.controller.linear_speed

    @linear_speed.setter
    def linear_speed(self, speed):
        """Sets the linear speed of the robot TCP (mm/s).
        """
        self.controller.linear_speed = speed

    @property
    def angular_speed(self):
        """Returns the angular speed of the robot TCP (deg/s).
        """
        return self.controller.angular_speed

    @angular_speed.setter
    def angular_speed(self, speed):
        """Sets the angular speed of the robot TCP (deg/s).
        """
        self.controller.angular_speed = speed

    @property
    def blend_radius(self):
        """Returns the robot blend radius (mm).
        """
        return self.controller.blend_radius

    @blend_radius.setter
    def blend_radius(self, blend_radius):
        """Sets the robot blend radius (mm).
        """
        self.controller.blend_radius = blend_radius

    @property
    def joint_angles(self):
        """ Returns the current joint angles.
        """
        return self.controller.joint_angles

    @property
    def target_joint_angles(self):
        """ Returns the target joint angles.
        """
        return self.controller.commanded_joint_angles

    @property
    def pose(self):
        """Returns the current TCP pose in the reference coordinate frame.
        """
        pose_q = self.controller.pose
        return quat2euler(transform(pose_q, self._coord_frame_q), self._axes)

    @property
    def target_pose(self):
        """Returns the target TCP pose in the reference coordinate frame.
        """
        pose_q = self.controller.commanded_pose
        return quat2euler(transform(pose_q, self._coord_frame_q), self._axes)

    @property
    def elbow(self):
        """Returns the current elbow angle (degrees).
        """
        return self.controller.elbow

    @property
    def target_elbow(self):
        """Returns the target elbow angle (degrees).
        """
        return self.controller.commanded_elbow

    def move_joints(self, joint_angles):
        """Executes an immediate move to the specified joint angles.
        """
        check_joint_angles(joint_angles)
        joint_angles = np.array(joint_angles)
        self.controller.move_joints(joint_angles)
    
    def move_linear(self, pose, elbow=None):
        """Executes a linear/cartesian move from the current TCP pose to the
        specified pose in the reference coordinate frame.
        """
        check_pose(pose)
        target_base_pose_q = inv_transform(euler2quat(pose, self._axes), self._coord_frame_q)
        self.controller.move_linear(target_base_pose_q, elbow)

    def move_circular(self, via_pose, end_pose, elbow=None):
        """Executes a movement in a circular path from the current TCP pose,
        through via_pose, to end_pose in the reference coordinate frame.
        """
        check_pose(via_pose)
        check_pose(end_pose)
        via_base_pose_q = inv_transform(euler2quat(via_pose, self._axes), self._coord_frame_q)
        target_base_pose_q = inv_transform(euler2quat(end_pose, self._axes), self._coord_frame_q)
        self.controller.move_circular(via_base_pose_q, target_base_pose_q, elbow)
   
    def close(self):
        """Releases any resources held by the robot (e.g., sockets).
        """
        self.controller.close()      


class AsyncRobot(Robot):
    """Asynchronous robot class provides synchronous (blocking) and
    asynchronous (non-blocking) movement primitives.
    """
    
    def __init__(self, sync_robot):
        self.sync_robot = sync_robot
        try:
            self._worker = None
            self._results = queue.Queue()
            self._busy = False
        except:
            self.sync_robot.close()
            raise

    @property
    def info(self):
        return self.sync_robot.info

    @property
    def axes(self):
        return self.sync_robot.axes

    @axes.setter
    def axes(self, axes):
        self.sync_robot.axes = axes

    @property
    def tcp(self):
        return self.sync_robot.tcp

    @tcp.setter
    def tcp(self, tcp):
        self.sync_robot.tcp = tcp

    @property
    def coord_frame(self):
        return self.sync_robot.coord_frame

    @coord_frame.setter
    def coord_frame(self, frame):
        self.sync_robot.coord_frame = frame

    @property
    def linear_speed(self):
        return self.sync_robot.linear_speed

    @linear_speed.setter
    def linear_speed(self, speed):
        self.sync_robot.linear_speed = speed

    @property
    def angular_speed(self):
        return self.sync_robot.angular_speed

    @angular_speed.setter
    def angular_speed(self, speed):
        self.sync_robot.angular_speed = speed

    @property
    def blend_radius(self):
        return self.sync_robot.blend_radius

    @blend_radius.setter
    def blend_radius(self, blend_radius):
        self.sync_robot.blend_radius = blend_radius

    @property
    def joint_angles(self):
        return self.sync_robot.joint_angles

    @property
    def target_joint_angles(self):
        return self.sync_robot.target_joint_angles

    @property
    def pose(self):
        return self.sync_robot.pose

    @property
    def target_pose(self):
        return self.sync_robot.target_pose

    @property
    def elbow(self):
        return self.sync_robot.elbow

    @property
    def target_elbow(self):
        return self.sync_robot.target_elbow

    def move_joints(self, joint_angles):
        self.sync_robot.move_joints(joint_angles)

    def move_linear(self, pose, elbow=None):
        self.sync_robot.move_linear(pose, elbow)

    def move_circular(self, via_pose, end_pose, elbow=None):
        self.sync_robot.move_circular(via_pose, end_pose, elbow)

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
                              results.put(self.sync_robot.move_joints(joint_angles)), \
                              args=(joint_angles, self._results))
        self._worker.start()
    
    def async_move_linear(self, pose, elbow=None):
        """Executes a linear/cartesian move from the current TCP pose to the
        specified pose in the reference coordinate frame.
        
        pose = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Cartesian position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        elbow = target elbow angle for 7-DOF robot arms (optional; degrees)
        """
        if self._busy:
            raise AsyncBusy   
        self._busy = True
        self._worker = Thread(target=lambda pose, elbow, results: \
                              results.put(self.sync_robot.move_linear(pose, elbow)), \
                              args=(pose, elbow, self._results))
        self._worker.start()

    def async_move_circular(self, via_pose, end_pose, elbow=None):
        """Executes a movement in a circular path from the current TCP pose,
        through via_pose, to end_pose in the reference coordinate frame.

        via_pose, end_pose = (x, y, z, alpha, beta, gamma)
        x, y, z specify a Cartesian position (mm)
        alpha, beta, gamma specify an euler rotation (degrees)
        elbow = target elbow angle for 7-DOF robot arms (optional; degrees)
        """
        if self._busy:
            raise AsyncBusy
        self._busy = True
        self._worker = Thread(target=lambda via_pose, end_pose, elbow, results: \
                              results.put(self.sync_robot.move_circular(via_pose, end_pose, elbow)),\
                              args=(via_pose, end_pose, elbow, self._results))
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
        self.sync_robot.close()
