# -*- coding: utf-8 -*-
"""Robot controller interface/implementations provide a common, low-level
interface to various robot arms.
"""

import warnings
from abc import ABC, abstractmethod

import numpy as np

from cri.transforms import quat2axangle, axangle2quat, quat2mat, mat2quat, \
    mat2euler, quat2euler, euler2quat, transform, inv_transform
from cri.abb.abb_client import ABBClient
from cri.ur.rtde_client import RTDEClient

try:
    import frankx
    import pyaffx
except ImportError:
    warnings.warn("Failed to import frankx library: frankx controller not available")


class RobotController(ABC):
    """Robot controller class provides a common interface to various robot arms.
    
    Poses and coordinate frames are specified using 3D Cartesian positions
    and quaternion rotations.  This makes it easy to perform coordinate
    transformations using quaternion operations.    
    """
    def __repr__(self):      
        return "{} ({})".format(self.__class__.__name__, self.info)

    def __str__(self):
        return self.__repr__()

    def __enter__(self):
        return self
        
    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
 
    @property       
    @abstractmethod
    def info(self):
        """Returns a unique robot identifier string.
        """
        pass

    @property    
    @abstractmethod
    def tcp(self):
        """Returns the tool center point (TCP) of the robot.
        
        The TCP is specified in the output flange frame, which is located at
        the intersection of the tool flange center axis and the flange face,
        with the z-axis aligned with the tool flange center axis.
        
        tool = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Cartesian position (mm)
        qw, qx, qy, qz specify a quaternion rotation
        """
        pass

    @tcp.setter
    @abstractmethod
    def tcp(self, tcp):
        """Sets the tool center point (TCP) of the robot.
        
        The TCP is specified in the output flange frame, which is located at
        the intersection of the tool flange center axis and the flange face,
        with the z-axis aligned with the tool flange center axis.
        
        tcp = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Cartesian position (mm)
        qw, qx, qy, qz specify a quaternion rotation
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
        """Returns the joint speed of the robot TCP (deg/s).
        """
        pass

    @angular_speed.setter    
    @abstractmethod
    def angular_speed(self, speed):
        """Sets the joint speed of the robot TCP (deg/s).
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
        """Returns the robot joint angles.
        
        joint angles = (j0, j1, j2, j3, j4, j5, [j6])
        j0, j1, j2, j3, j4, j5, [j6] are numbered from base to end effector and are
        measured in degrees
        """
        pass

    @property    
    @abstractmethod
    def pose(self):
        """Returns the TCP pose in the reference coordinate frame.
        
        pose = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Cartesian position (mm)
        qw, qx, qy, qz specify a quaternion rotation
        """
        pass

    @property
    @abstractmethod
    def elbow(self):
        """Returns the current elbow angle.
        """
        pass

    @abstractmethod
    def move_joints(self, joint_angles):
        """Executes an immediate move to the specified joint angles.
        
        joint_angles = (j0, j1, j2, j3, j4, j5)
        j0, j1, j2, j3, j4, j5 are numbered from base to end effector and are
        measured in degrees
        """
        pass

    @abstractmethod
    def move_linear(self, pose, elbow):
        """Executes a linear/cartesian move from the current base frame pose to
        the specified pose.
        
        pose = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Cartesian position (mm)
        qw, qx, qy, qz specify a quaternion rotation
        elbow = target elbow angle for 7-DOF robot arm (optional)
        """
        pass

    @abstractmethod
    def move_circular(self, via_pose, end_pose, elbow):
        """Executes a movement in a circular path from the current base frame
        pose, through via_pose, to end_pose.
        
        via_pose, end_pose = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Cartesian position (mm)
        qw, qx, qy, qz specify a quaternion rotation
        elbow = target elbow angle for 7-DOF robot arm (optional)
        """
        pass

    @abstractmethod        
    def close(self):
        """Releases any resources held by the controller (e.g., sockets).
        """
        pass

   
class ABBController(RobotController):
    """ABB controller class implements common interface robot arms.
    
    Currently implemented using an adapted version of the Python 2 client
    example in OpenABB project (https://github.com/robotics/open_abb), which
    has been modified to run under Python 3.
    
    Poses and coordinate frames are specified using 3D Cartesian positions
    and a quaternion rotations.  This format makes it easy to perform
    coordinate transformations.
    """
    def __init__(self, ip='192.168.125.1', port=5000):
        self._ip = ip
        self._port = port
        self._client = ABBClient(ip, port)
        try:
            self.tcp = (0, 0, 0, 1, 0, 0, 0)    # base frame (quaternion)
            self.linear_speed = 20              # mm/s
            self.angular_speed = 20             # deg/s
            self.blend_radius = 0               # mm
        except:
            self._client.close()
            raise

    @property
    def info(self):
        """Returns a unique robot identifier string.
        """
        return "ip: {}, port: {}, info: {}".format(
                self._ip,
                self._port,
                self._client.get_info(),
                )

    @property    
    def tcp(self):
        """Returns the tool center point (TCP) of the robot.
        """
        return self._tcp

    @tcp.setter    
    def tcp(self, tcp):
        """Sets the tool center point (TCP) of the robot.
        """
        self._client.set_tcp(tcp)
        self._tcp = tcp

    @property
    def linear_speed(self):
        """Returns the linear speed of the robot TCP (mm/s).
        """
        return self._linear_speed

    @linear_speed.setter
    def linear_speed(self, speed):
        """Sets the linear speed of the robot TCP (mm/s).
        """
        try:
            self._angular_speed
        except AttributeError:
            self._client.set_speed(linear_speed=speed,
                                   angular_speed=20)
        else:
            self._client.set_speed(linear_speed=speed,
                                   angular_speed=self._angular_speed)
        self._linear_speed = speed

    @property
    def angular_speed(self):
        """Returns the angular speed of the robot TCP (deg/s).
        """
        return self._angular_speed

    @angular_speed.setter
    def angular_speed(self, speed):
        """Sets the angular speed of the robot TCP (deg/s).
        """
        try:
            self._linear_speed
        except AttributeError:
            self._client.set_speed(linear_speed=20,
                                   angular_speed=speed)
        else:
            self._client.set_speed(linear_speed=self._linear_speed,
                                   angular_speed=speed)
        self._angular_speed = speed

    @property
    def blend_radius(self):
        """Returns the robot blend radius (mm).
        """
        return self._blend_radius

    @blend_radius.setter
    def blend_radius(self, blend_radius):
        """Sets the robot blend radius (mm).
        """
        if blend_radius == 0:
            self._client.set_zone(point_motion=True,
                                  manual_zone=(blend_radius,)*3)
        else:
            self._client.set_zone(point_motion=False,
                                  manual_zone=(blend_radius,)*3)
        self._blend_radius = blend_radius

    @property
    def joint_angles(self):
        """Returns the robot joint angles.
        """
        return self._client.get_joint_angles()

    @property
    def pose(self):
        """Returns the TCP pose in the reference coordinate frame.
        """
        return self._client.get_pose()

    @property
    def elbow(self):
        """Returns the current elbow angle.
        """
        warnings.warn("elbow property not implemented in ABB controller")
        return None

    def move_joints(self, joint_angles):
        """Executes an immediate move to the specified joint angles.
        """
        self._client.move_joints(joint_angles)

    def move_linear(self, pose, elbow=None):
        """Executes a linear/cartesian move from the current base frame pose to
        the specified pose.
        """
        if elbow is not None:
            warnings.warn("elbow property not implemented in ABB controller")
        self._client.move_linear(pose)

    def move_circular(self, via_pose, end_pose, elbow=None):
        """Executes a movement in a circular path from the current base frame
        pose, through via_pose, to end_pose.
        """
        if elbow is not None:
            warnings.warn("elbow property not implemented in ABB controller")
        self._client.move_circular(via_pose, end_pose)

    def close(self):
        """Releases any resources held by the controller (e.g., sockets).
        """
        self._client.close()


class RTDEController(RobotController):
    """UR RTDE controller class implements common interface to robot arms.
    
    Poses and coordinate frames are specified using 3D Cartesian positions
    and quaternion rotations.  This format makes it easy to perform
    coordinate transformations.
    """
    def __init__(self, ip='192.168.125.1'):
        self._ip = ip
        self._client = RTDEClient(ip)        
        try:   
            self.tcp = (0, 0, 0, 1, 0, 0, 0)    # base frame (quaternion)
            self.linear_accel = 500             # mm/s/s
            self.linear_speed = 20              # mm/s
            self.angular_accel = 50             # deg/s/s
            self.angular_speed = 20             # deg/s
            self.blend_radius = 0               # mm
        except:
            self._client.close()
            raise

    @property    
    def info(self):
        """Returns a unique robot identifier string.
        """
        return "ip: {}, info: {}".format(self._ip, self._client.get_info())

    @property    
    def tcp(self):
        """Returns the tool center point (TCP) of the robot.
        """
        return self._tcp

    @tcp.setter    
    def tcp(self, tcp):
        """Sets the tool center point (TCP) of the robot.
        """
        self._client.set_tcp(quat2axangle(tcp))
        self._tcp = tcp

    @property
    def linear_accel(self):
        """Returns the linear acceleration of the robot TCP (mm/s/s).
        """
        return self._linear_accel
    
    @linear_accel.setter    
    def linear_accel(self, accel):
        """Sets the linear acceleration of the robot TCP (mm/s/s).
        """
        self._client.set_linear_accel(accel)
        self._linear_accel = accel

    @property   
    def linear_speed(self):
        """Returns the linear speed of the robot TCP (mm/s).
        """
        return self._linear_speed

    @linear_speed.setter    
    def linear_speed(self, speed):
        """Sets the linear speed of the robot TCP (mm/s).
        """
        self._client.set_linear_speed(speed)
        self._linear_speed = speed

    @property
    def angular_accel(self):
        """Returns the angular acceleration of the robot TCP (deg/s/s).
        """
        return self._angular_accel

    @angular_accel.setter
    def angular_accel(self, accel):
        """Sets the angular acceleration of the robot TCP (deg/s/s).
        """
        self._client.set_angular_accel(accel)
        self._angular_accel = accel

    @property
    def angular_speed(self):
        """Returns the angular speed of the robot TCP (deg/s).
        """
        return self._angular_speed

    @angular_speed.setter
    def angular_speed(self, speed):
        """Sets the angular speed of the robot TCP (deg/s).
        """
        self._client.set_angular_speed(speed)
        self._angular_speed = speed

    @property
    def blend_radius(self):
        """Returns the robot blend radius (mm).
        """
        return self._blend_radius

    @blend_radius.setter    
    def blend_radius(self, blend_radius):
        """Sets the robot blend radius (mm).
        """
        self._client.set_blend_radius(blend_radius)
        self._blend_radius = blend_radius

    @property
    def joint_angles(self):
        """Returns the robot joint angles.
        """
        return self._client.get_joint_angles()

    @property
    def pose(self):
        """Returns the current base frame pose.
        """
        return axangle2quat(self._client.get_pose())  

    @property
    def elbow(self):
        """Returns the current elbow angle.
        """
        warnings.warn("elbow property not implemented in RTDE controller")
        return None

    def move_joints(self, joint_angles):
        """Executes an immediate move to the specified joint angles.
        """
        self._client.move_joints(joint_angles)

    def move_linear(self, pose, elbow=None):
        """Executes a linear/cartesian move from the current base frame pose to
        the specified pose.
        """
        if elbow is not None:
            warnings.warn("elbow property not implemented in RTDE controller")
        self._client.move_linear(quat2axangle(pose))

    def move_circular(self, via_pose, end_pose, elbow=None):
        """Executes a movement in a circular path from the current base frame
        pose, through via_pose, to end_pose.
        """
        if elbow is not None:
            warnings.warn("elbow property not implemented in RTDE controller")
        self._client.move_circular(quat2axangle(via_pose),
                                   quat2axangle(end_pose))
      
    def close(self):
        """Releases any resources held by the controller (e.g., sockets).
        """
        return self._client.close()


class FrankxController(RobotController):
    """Frankx controller class implements common interface to robot arms.

    Poses and coordinate frames are specified using 3D Cartesian positions
    and quaternion rotations.  This format makes it easy to perform
    coordinate transformations.

    An offset frame is used to make sure that the poses sent to the
    underlying trajectory generator are in a continuous region of Euler
    space.
    """

    MAX_MOVE_ATTEMPTS = 3

    def __init__(self, ip='172.16.0.2'):
        self._ip = ip
        self._client = frankx.Robot(ip)
        self._client.recover_from_errors()
        self._client.set_default_behavior()

        self._tcp = euler2quat((0, 0, 0, 0, 0, 0))
        self._offset = euler2quat((0, 0, 0, 180, 0, 180))

        self.set_units('millimeters', 'degrees')
        self._set_EE()

        self.rel_velocity = 0.4
        self.rel_accel = 0.03
        self.rel_jerk = 0.02

    def _set_EE(self):
        ne_t_ee = inv_transform(self._offset, self._tcp)
        ne_t_ee[:3] *= self._scale_linear
        self._client.set_EE(quat2mat(ne_t_ee).flatten(order='F'))

    @property
    def info(self):
        """Returns a unique robot identifier string.
        """
        # return "ip: {}, info: {}".format(self._ip, self._client.server_version())
        return "ip: {}".format(self._ip)

    @property
    def tcp(self):
        """Returns the tool center point (TCP) of the robot.
        """
        return self._tcp

    @tcp.setter
    def tcp(self, tcp):
        """Sets the tool center point (TCP) of the robot.
        """
        self._tcp = np.array(tcp, dtype=np.float64).ravel()
        self._set_EE()

    @property
    def offset(self):
        """Returns the tool center point (TCP) offset of the robot
        """
        return self._offset

    @offset.setter
    def offset(self, offset):
        """Sets the tool center point (TCP) offset of the robot
        """
        self._offset = np.array(offset, dtype=np.float64).ravel()
        self._set_EE()

    @property
    def max_trans_velocity(self):
        """Returns the maximum translational velocity of the robot TCP (mm/s).
        """
        return self._client.max_translation_velocity / self._scale_linear

    @property
    def max_rot_velocity(self):
        """Returns the maximum rotational velocity of the robot TCP (deg/s).
        """
        return self._client.max_rotation_velocity / self._scale_angle

    @property
    def max_elbow_velocity(self):
        """Returns the maximum elbow velocity of the robot (deg/s).
        """
        return self._client.max_elbow_velocity / self._scale_angle

    @property
    def max_trans_accel(self):
        """Returns the maximum translational acceleration of the robot TCP (mm/s/s).
        """
        return self._client.max_translation_acceleration / self._scale_linear

    @property
    def max_rot_accel(self):
        """Returns the maximum rotational acceleration of the robot TCP (deg/s/s).
        """
        return self._client.max_rotation_acceleration / self._scale_angle

    @property
    def max_elbow_accel(self):
        """Returns the maximum elbow acceleration of the robot (deg/s/s).
        """
        return self._client.max_elbow_acceleration / self._scale_angle

    @property
    def max_trans_jerk(self):
        """Returns the maximum translational jerk of the robot TCP (mm/s/s/s).
        """
        return self._client.max_translation_jerk / self._scale_linear

    @property
    def max_rot_jerk(self):
        """Returns the maximum rotational jerk of the robot TCP (deg/s/s/s).
        """
        return self._client.max_rotation_jerk / self._scale_angle

    @property
    def max_elbow_jerk(self):
        """Returns the maximum elbow jerk of the robot TCP (deg/s/s/s).
        """
        return self._client.max_elbow_jerk / self._scale_linear

    @property
    def rel_velocity(self):
        """Returns the relative velocity of the robot
        """
        return self._rel_velocity

    @rel_velocity.setter
    def rel_velocity(self, rel_velocity):
        """Sets the velocity to rel_velocity % of the maximum (0 < rel_velocity <= 1)
        """
        self._rel_velocity = rel_velocity
        self._client.velocity_rel = rel_velocity

    @property
    def rel_accel(self):
        """Returns the relative acceleration of the robot
        """
        return self._rel_accel

    @rel_accel.setter
    def rel_accel(self, rel_accel):
        """Sets the acceleraton to rel_accel % of the maximum (0 < rel_accel <= 1)
        """
        self._rel_accel = rel_accel
        self._client.acceleration_rel = rel_accel

    @property
    def rel_jerk(self):
        """Returns the relative jerk of the robot
        """
        return self._rel_jerk

    @rel_jerk.setter
    def rel_jerk(self, rel_jerk):
        """Sets the jerk to rel_jerk % of the maximum (0 < rel_jerk <= 1)
        """
        self._rel_jerk = rel_jerk
        self._client.jerk_rel = rel_jerk

    @property
    def linear_accel(self):
        """Returns the linear acceleration of the robot TCP (mm/s/s).
        """
        warnings.warn("linear_accel property not implemented in frankx")
        return None

    @linear_accel.setter
    def linear_accel(self, accel):
        """Sets the linear acceleration of the robot TCP (mm/s/s).
        """
        warnings.warn("linear_accel property not implemented in frankx")

    @property
    def linear_speed(self):
        """Returns the linear speed of the robot TCP (mm/s).
        """
        warnings.warn("linear_speed property not implemented in frankx")
        return None

    @linear_speed.setter
    def linear_speed(self, speed):
        """Sets the linear speed of the robot TCP (mm/s).
        """
        warnings.warn("linear_speed property not implemented in frankx")

    @property
    def angular_accel(self):
        """Returns the angular acceleration of the robot TCP (deg/s/s).
        """
        warnings.warn("angular_accel property not implemented in frankx")
        return None

    @angular_accel.setter
    def angular_accel(self, accel):
        """Sets the angular acceleration of the robot TCP (deg/s/s).
        """
        self._client.set_angular_accel(accel)
        warnings.warn("angular_accel property not implemented in frankx")

    @property
    def angular_speed(self):
        """Returns the angular speed of the robot TCP (deg/s).
        """
        warnings.warn("angular_speed property not implemented in frankx")
        return None

    @angular_speed.setter
    def angular_speed(self, speed):
        """Sets the angular speed of the robot TCP (deg/s).
        """
        warnings.warn("angular_speed property not implemented in frankx")

    @property
    def blend_radius(self):
        """Returns the robot blend radius (mm).
        """
        warnings.warn("blend_radius property not implemented in frankx")
        return None

    @blend_radius.setter
    def blend_radius(self, blend_radius):
        """Sets the robot blend radius (mm).
        """
        warnings.warn("blend_radius property not implemented in frankx")

    def set_units(self, linear, angular):
        """Sets linear and angular units.
        """
        units_l = {'millimeters' : 0.001,
                   'meters' : 1.0,
                   'inches' : 0.0254,
                   }
        units_a = {'degrees' : 0.0174533,
                   'radians' : 1.0,
                   }
        self._scale_linear = units_l[linear]
        self._scale_angle  = units_a[angular]

    @property
    def joint_angles(self):
        """Returns the robot joint angles.
        """
        state = self._client.read_once()
        joint_angles = np.array(state.q, dtype=np.float64)
        joint_angles /= self._scale_angle
        return joint_angles

    @property
    def pose(self):
        """Returns the current base frame pose.
        """
        state = self._client.read_once()
        pose = np.array(state.O_T_EE, dtype=np.float64)
        pose = mat2quat(pose.reshape((4, 4), order='F'))
        inv_offset = inv_transform(euler2quat((0, 0, 0, 0, 0, 0)), self._offset)
        pose = inv_transform(inv_offset, pose)
        pose[:3] /= self._scale_linear
        return pose

    @property
    def elbow(self):
        """Returns the current elbow angle.
        """
        state = self._client.read_once()
        elbow = state.elbow[0]
        elbow /= self._scale_angle
        return elbow

    def move_joints(self, joint_angles):
        """Executes an immediate move to the specified joint angles.
        """
        joint_angles = np.array(joint_angles, dtype=np.float64).ravel()
        joint_angles *= self._scale_angle
        motion = frankx.JointMotion(joint_angles)
        for attempt in range(self.MAX_MOVE_ATTEMPTS):
            if self._client.move(motion):
                break
            if attempt < self.MAX_MOVE_ATTEMPTS - 1:
                warnings.warn("Move failed - trying again ...")
            else:
                warnings.warn("Move failed - aborting")
            self._client.recover_from_errors()

    def move_linear(self, pose, elbow=None):
        """Executes a linear/cartesian move from the current base frame pose to
        the specified pose, using the optional target elbow angle.
        """
        pose = inv_transform(self._offset, pose)
        pose = quat2euler(pose, axes='rzyx')
        pose[:3] *= self._scale_linear
        pose[3:] *= self._scale_angle
        if elbow is None:
            motion = frankx.LinearMotion(pyaffx.Affine(*pose))
        else:
            elbow *= self._scale_angle
            motion = frankx.LinearMotion(pyaffx.Affine(*pose), elbow)
        for attempt in range(self.MAX_MOVE_ATTEMPTS):
            if self._client.move(motion):
                break
            if attempt < self.MAX_MOVE_ATTEMPTS - 1:
                warnings.warn("Move failed - trying again ...")
            else:
                warnings.warn("Move failed - aborting")
            self._client.recover_from_errors()

    def move_circular(self, via_pose, end_pose, elbow=None):
        """Executes a movement in a circular path from the current base frame
        pose, through via_pose, to end_pose.
        """
        raise NotImplementedError

    def recover_from_errors(self):
        """Recover from errors and reset Franka controller.
        """
        self._client.recover_from_errors()

    def close(self):
        """Recover from errors and reset Franka controller.
        """
        self._client.recover_from_errors()
