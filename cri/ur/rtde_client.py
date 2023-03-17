# -*- coding: utf-8 -*-
"""Python client interface for UR5 RTDE server.
"""

import os

import numpy as np

import cri.ur.rtde.rtde_proxy as rtde_proxy
import cri.ur.rtde.rtde_config as rtde_config


class RTDEClient:
    """Python client interface for UR5 RTDE server.
    """
    class RTDEProtocolNotSupported(RuntimeError):
        pass
    
    class RTDESyncFailedToStart(RuntimeError):
        pass
    
    def __init__(self, ip='164.11.72.164'):
        self.set_units('millimeters', 'degrees')
        self.connect(ip, port=30004)

    def __repr__(self):      
        return "{} ({})".format(self.__class__.__name__, self.get_info())
        
    def __str__(self):
        return self.__repr__()

    def __enter__(self):
        return self
        
    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def _wait_for_command_complete(self):
        """Handles command completion handshaking with server.
        """
        # wait for command complete signal from server
        self._state = self._con.receive()
        command_status = self._state.output_int_register_0
        while command_status == 0:
            self._state = self._con.receive()
            command_status = self._state.output_int_register_0
        # send ack to server (reset command = 0)
        self._command.input_int_register_0 = 0
        self._con.send(self._command)
        # wait for command complete signal reset
        self._state = self._con.receive()
        command_status = self._state.output_int_register_0
        while command_status != 0:
            self._state = self._con.receive()
            command_status = self._state.output_int_register_0

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

    def connect(self, ip, port):
        """Initializes RTDE interface and connects to server.
        """
        # get RTDE recipe definitions
        config_file = os.path.join(os.path.dirname(__file__), "rtde_config.xml")
        conf = rtde_config.ConfigFile(config_file)
        state_names, state_types = conf.get_recipe('state')
        command_names, command_types = conf.get_recipe('command')
        params_vec_6d_names, params_vec_6d_types = conf.get_recipe('params_vec_6d')          
        params_vec_6d_2_names, params_vec_6d_2_types = conf.get_recipe('params_vec_6d_2') 
        params_1d_names, params_1d_types = conf.get_recipe('params_1d')
        params_1d_2_names, params_1d_2_types = conf.get_recipe('params_1d_2')

        # connect to RTDE interface
        self._con = rtde_proxy.RTDEProxy(hostname=ip, port=port)
        self._con.connect()
        if not self._con.negotiate_protocol_version(protocol=1):
            raise RTDEClient.RTDEProtocolNotSupported 

        # set up RTDE recipes
        self._con.send_output_setup(state_names, state_types)
        self._command = self._con.send_input_setup(command_names, command_types)
        self._params_vec_6d = self._con.send_input_setup(params_vec_6d_names, params_vec_6d_types)
        self._params_vec_6d_2 = self._con.send_input_setup(params_vec_6d_2_names, params_vec_6d_2_types)        
        self._params_1d = self._con.send_input_setup(params_1d_names, params_1d_types)
        self._params_1d_2 = self._con.send_input_setup(params_1d_2_names, params_1d_2_types)
      
        # start RTDE interface
        if not self._con.send_start():
            raise RTDEClient.RTDESyncFailedToStart
      
        # initialize command (0 = none)
        self._command.input_int_register_0 = 0
        self._con.send(self._command)

        # get initial server state
        self._state = self._con.receive()

    def move_joints(self, joint_angles):
        """Executes an immediate move to the specified joint angles.
        
        joint_angles = (j0, j1, j2, j3, j4, j5)
        j0, j1, j2, j3, j4, j5 are numbered from base to end effector and are
        measured in degrees
        """
        joint_angles = np.array(joint_angles, dtype=np.float64).ravel()
        joint_angles *= self._scale_angle
        
        self._command.input_int_register_0 = 1
        self._params_vec_6d.input_double_register_0 = joint_angles[0]
        self._params_vec_6d.input_double_register_1 = joint_angles[1]
        self._params_vec_6d.input_double_register_2 = joint_angles[2]
        self._params_vec_6d.input_double_register_3 = joint_angles[3]
        self._params_vec_6d.input_double_register_4 = joint_angles[4]
        self._params_vec_6d.input_double_register_5 = joint_angles[5]

        self._con.send(self._params_vec_6d)
        self._con.send(self._command)
        self._wait_for_command_complete()
        
    def move_linear(self, pose):
        """Executes a linear/cartesian move from the current base frame pose to
        the specified pose.
        
        pose = (x, y, z, ax, ay, az)
        x, y, z specify a Cartesian position (mm)
        ax, ay, az specify an axis-angle rotation (rad)
        """
        pose = np.array(pose, dtype=np.float64).ravel()
        pose[:3] *= self._scale_linear
        
        self._command.input_int_register_0 = 2
        self._params_vec_6d.input_double_register_0 = pose[0]
        self._params_vec_6d.input_double_register_1 = pose[1]
        self._params_vec_6d.input_double_register_2 = pose[2]
        self._params_vec_6d.input_double_register_3 = pose[3]
        self._params_vec_6d.input_double_register_4 = pose[4]
        self._params_vec_6d.input_double_register_5 = pose[5]

        self._con.send(self._params_vec_6d)
        self._con.send(self._command)
        self._wait_for_command_complete()

    def move_circular(self, via_pose, end_pose):
        """Executes a movement in a circular path from the current base frame
        pose, through via_pose, to end_pose.
        
        via_pose, end_pose = (x, y, z, ax, ay, az)
        x, y, z specify a Cartesian position (mm)
        ax, ay, az specify an axis-angle rotation (rad)
        """        
        via_pose = np.array(via_pose, dtype=np.float64).ravel()
        via_pose[:3] *= self._scale_linear
        end_pose = np.array(end_pose, dtype=np.float64).ravel()
        end_pose[:3] *= self._scale_linear        
        
        self._command.input_int_register_0 = 3
        self._params_vec_6d_2.input_double_register_6 = via_pose[0]
        self._params_vec_6d_2.input_double_register_7 = via_pose[1]
        self._params_vec_6d_2.input_double_register_8 = via_pose[2]
        self._params_vec_6d_2.input_double_register_9 = via_pose[3]
        self._params_vec_6d_2.input_double_register_10 = via_pose[4]
        self._params_vec_6d_2.input_double_register_11 = via_pose[5]
        self._params_vec_6d_2.input_double_register_12 = end_pose[0]
        self._params_vec_6d_2.input_double_register_13 = end_pose[1]
        self._params_vec_6d_2.input_double_register_14 = end_pose[2]
        self._params_vec_6d_2.input_double_register_15 = end_pose[3]
        self._params_vec_6d_2.input_double_register_16 = end_pose[4]
        self._params_vec_6d_2.input_double_register_17 = end_pose[5]

        self._con.send(self._params_vec_6d_2)
        self._con.send(self._command)
        self._wait_for_command_complete()

    def move_joint_speed(self, joint_speeds, joint_accel, return_time=None):
        """Accelerate linearly in joint space and continue with constant joint
        speed. The return time is optional; if provided the function will
        return after that time, regardless of the target speed has been reached.
        If the return time is not provided, the function will return when the
        target speed is reached.

        joint_speeds = (jd0, jd1, jd2, jd3, jd4, jd5)
        jd0, jd1, jd2, jd3, jd4, jd5 are numbered from base to end effector and are
        measured in deg/s
        joint_accel is measured in deg/s/s (of leading axis)
        return_time is measured in secs before the function returns (optional)
        """
        joint_speeds = np.array(joint_speeds, dtype=np.float64).ravel()
        joint_speeds *= self._scale_angle
        joint_accel *= self._scale_angle
        return_time = -1 if return_time is None else return_time

        self._command.input_int_register_0 = 10
        self._params_vec_6d.input_double_register_0 = joint_speeds[0]
        self._params_vec_6d.input_double_register_1 = joint_speeds[1]
        self._params_vec_6d.input_double_register_2 = joint_speeds[2]
        self._params_vec_6d.input_double_register_3 = joint_speeds[3]
        self._params_vec_6d.input_double_register_4 = joint_speeds[4]
        self._params_vec_6d.input_double_register_5 = joint_speeds[5]
        self._params_1d_2.input_double_register_19 = joint_accel
        self._params_1d_2.input_double_register_20 = return_time

        self._con.send(self._params_vec_6d)
        self._con.send(self._params_1d_2)
        self._con.send(self._command)
        self._wait_for_command_complete()

    def move_linear_speed(self, linear_speed, linear_accel, return_time=None):
        """Accelerate linearly in Cartesian space and continue with constant tool
        speed. The return time is optional; if provided the function will return after
        that time, regardless of the target speed has been reached. If the return time
        is not provided, the function will return when the target speed is reached

        linear speed = (xd, yd, zd, axd, ayd, azd)
        xd, yd, zd specify a translational velocity (mm/s)
        axd, ayd, azd specify an axis-angle rotational velocity (deg/s)
        linear_accel is measured in mm/s/s
        return_time is measured in secs before the function returns (optional)
        """
        linear_speed = np.array(linear_speed, dtype=np.float64).ravel()
        linear_speed[:3] *= self._scale_linear
        linear_speed[3:] *= self._scale_angle
        linear_accel *= self._scale_linear
        return_time = -1 if return_time is None else return_time

        self._command.input_int_register_0 = 11
        self._params_vec_6d.input_double_register_0 = linear_speed[0]
        self._params_vec_6d.input_double_register_1 = linear_speed[1]
        self._params_vec_6d.input_double_register_2 = linear_speed[2]
        self._params_vec_6d.input_double_register_3 = linear_speed[3]
        self._params_vec_6d.input_double_register_4 = linear_speed[4]
        self._params_vec_6d.input_double_register_5 = linear_speed[5]
        self._params_1d_2.input_double_register_19 = linear_accel
        self._params_1d_2.input_double_register_20 = return_time

        self._con.send(self._params_vec_6d)
        self._con.send(self._params_1d_2)
        self._con.send(self._command)
        self._wait_for_command_complete()

    def stop_joints(self, joint_accel):
        """Decelerate joint speeds to zero.

        joint_accel is measured in deg/s/s (of leading axis)
        """
        joint_accel *= self._scale_angle
        self._command.input_int_register_0 = 12
        self._params_1d.input_double_register_18 = joint_accel
        self._con.send(self._params_1d)
        self._con.send(self._command)
        self._wait_for_command_complete()

    def stop_linear(self, linear_accel):
        """Decelerate linear speed to zero.

        linear_accel is measured in mm/s/s
        """
        linear_accel *= self._scale_linear
        self._command.input_int_register_0 = 13
        self._params_1d.input_double_register_18 = linear_accel
        self._con.send(self._params_1d)
        self._con.send(self._command)
        self._wait_for_command_complete()

    def set_tcp(self, tcp):
        """Sets the tool center point (TCP) of the robot.
        
        The TCP is specified in the output flange frame, which is located at
        the intersection of the tool flange center axis and the flange face,
        with the z-axis aligned with the tool flange center axis.
        
        tcp = (x, y, z, ax, ay, az)
        x, y, z specify a Cartesian position (mm)
        ax, ay, az specify an axis-angle rotation
        """
        tcp = np.array(tcp, dtype=np.float64).ravel()
        tcp[:3] *= self._scale_linear
        
        self._command.input_int_register_0 = 4
        self._params_vec_6d.input_double_register_0 = tcp[0]
        self._params_vec_6d.input_double_register_1 = tcp[1]
        self._params_vec_6d.input_double_register_2 = tcp[2]
        self._params_vec_6d.input_double_register_3 = tcp[3]
        self._params_vec_6d.input_double_register_4 = tcp[4]
        self._params_vec_6d.input_double_register_5 = tcp[5]

        self._con.send(self._params_vec_6d)
        self._con.send(self._command)
        self._wait_for_command_complete()

    def set_linear_accel(self, accel):
        """Sets the linear acceleration of the robot TCP (mm/s/s).
        """
        accel *= self._scale_linear        
        self._command.input_int_register_0 = 5
        self._params_1d.input_double_register_18 = accel
        self._con.send(self._params_1d)
        self._con.send(self._command)
        self._wait_for_command_complete()

    def set_linear_speed(self, speed):
        """Sets the linear speed of the robot TCP (mm/s).
        """
        speed *= self._scale_linear        
        self._command.input_int_register_0 = 6
        self._params_1d.input_double_register_18 = speed
        self._con.send(self._params_1d)
        self._con.send(self._command)
        self._wait_for_command_complete()

    def set_angular_accel(self, accel):
        """Sets the angular acceleration of the robot TCP (deg/s/s).
        """
        accel *= self._scale_angle
        self._command.input_int_register_0 = 7        
        self._params_1d.input_double_register_18 = accel
        self._con.send(self._params_1d)
        self._con.send(self._command)
        self._wait_for_command_complete()

    def set_angular_speed(self, speed):
        """Sets the angular speed of the robot TCP (deg/s).
        """
        speed *= self._scale_angle
        self._command.input_int_register_0 = 8      
        self._params_1d.input_double_register_18 = speed
        self._con.send(self._params_1d)
        self._con.send(self._command)
        self._wait_for_command_complete()
        
    def set_blend_radius(self, radius):
        """Sets the robot blend radius (mm).
        """
        radius *= self._scale_linear
        self._command.input_int_register_0 = 9        
        self._params_1d.input_double_register_18 = radius
        self._con.send(self._params_1d)
        self._con.send(self._command)
        self._wait_for_command_complete()

    def get_joint_angles(self):
        """Returns the current joint angles.
        
        joint_angles = (j0, j1, j2, j3, j4, j5)
        j0, j1, j2, j3, j4, j5 are numbered from base to end effector and are
        measured in degrees
        """
        self._state = self._con.receive()
        joint_angles = np.array(self._state.actual_q, dtype=np.float64)
        joint_angles /= self._scale_angle
        return joint_angles

    def get_target_joint_angles(self):
        """Returns the target joint angles.

        joint_angles = (j0, j1, j2, j3, j4, j5)
        j0, j1, j2, j3, j4, j5 are numbered from base to end effector and are
        measured in degrees
        """
        self._state = self._con.receive()
        joint_angles = np.array(self._state.target_q, dtype=np.float64)
        joint_angles /= self._scale_angle
        return joint_angles

    def get_pose(self):
        """Returns the current TCP pose in the reference coordinate frame.
        
        pose = (x, y, z, ax, ay, az)
        x, y, z specify a Cartesian position (mm)
        ax, ay, az specify an axis-angle rotation
        """
        self._state = self._con.receive()
        pose = np.array(self._state.actual_TCP_pose, dtype=np.float64)
        pose[:3] /= self._scale_linear      
        return pose

    def get_target_pose(self):
        """Returns the target TCP pose in the reference coordinate frame.

        pose = (x, y, z, ax, ay, az)
        x, y, z specify a Cartesian position (mm)
        ax, ay, az specify an axis-angle rotation
        """
        self._state = self._con.receive()
        pose = np.array(self._state.target_TCP_pose, dtype=np.float64)
        pose[:3] /= self._scale_linear
        return pose

    def get_joint_speeds(self):
        """Returns the current joint speeds.

        joint_speeds = (jd0, jd1, jd2, jd3, jd4, jd5)
        jd0, jd1, jd2, jd3, jd4, jd5 are numbered from base to end effector and are
        measured in deg/s
        """
        self._state = self._con.receive()
        joint_speeds = np.array(self._state.actual_qd, dtype=np.float64)
        joint_speeds /= self._scale_angle
        return joint_speeds

    def get_target_joint_speeds(self):
        """Returns the target joint speeds.

        joint_speeds = (jd0, jd1, jd2, jd3, jd4, jd5)
        jd0, jd1, jd2, jd3, jd4, jd5 are numbered from base to end effector and are
        measured in deg/s
        """
        self._state = self._con.receive()
        joint_speeds = np.array(self._state.target_qd, dtype=np.float64)
        joint_speeds /= self._scale_angle
        return joint_speeds

    def get_linear_speed(self):
        """Returns the current linear speed in the reference coordinate frame.

        linear speed = (xd, yd, zd, axd, ayd, azd)
        xd, yd, zd specify a translational velocity (mm/s)
        axd, ayd, azd specify an axis-angle rotational velocity (deg/s)
        """
        self._state = self._con.receive()
        linear_speed = np.array(self._state.actual_TCP_speed, dtype=np.float64)
        linear_speed[:3] /= self._scale_linear
        linear_speed[3:] /= self._scale_angle
        return linear_speed

    def get_target_linear_speed(self):
        """Returns the target linear speed in the reference coordinate frame.

        linear speed = (xd, yd, zd, axd, ayd, azd)
        xd, yd, zd specify a translational velocity (mm/s)
        axd, ayd, azd specify an axis-angle rotational velocity (deg/s)
        """
        self._state = self._con.receive()
        linear_speed = np.array(self._state.target_TCP_speed, dtype=np.float64)
        linear_speed[:3] /= self._scale_linear
        linear_speed[3:] /= self._scale_angle
        return linear_speed

    def get_info(self):
        """Returns a unique robot identifier string.
        """
        return "UR5 (controller version = {})".format(self._con.get_controller_version())

    def close(self):
        """Releases any resources held by the controller (e.g., sockets).
        """
        self._con.disconnect()
