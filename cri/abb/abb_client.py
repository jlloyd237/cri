# -*- coding: utf-8 -*-
"""Python client interface for ABB RAPID server.

Adapted from Open ABB python client (https://github.com/robotics/open_abb)
"""

import time
import socket
from struct import pack, unpack_from, calcsize

import numpy as np


class ABBClient:
    """Python client interface for ABB RAPID server.
    """
    class CommandFailed(RuntimeError):
        pass

    class InvalidZone(ValueError):
        pass
    
    SERVER_ERROR = 0
    SERVER_OK = 1
    
    def __init__(self, ip='192.168.125.1', port=5000):
        self._delay   = .08

        self.set_units('millimeters', 'degrees')
        self.connect((ip, port))

    def __repr__(self):      
        return "{} ({})".format(self.__class__.__name__, self.get_info())
        
    def __str__(self):
        return self.__repr__()
        
    def __enter__(self):
        return self
        
    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def set_units(self, linear, angular):
        """Sets linear and angular units.
        """
        units_l = {'millimeters' : 1.0,
                   'meters' : 1000.0,
                   'inches' : 25.4,
                   }
        units_a = {'degrees' : 1.0,
                   'radians' : 57.2957795,
                   }
        self._scale_linear = units_l[linear]
        self._scale_angle  = units_a[angular]
        
    def connect(self, remote):
        """Connects to server.
        """
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2.5)
        self.sock.connect(remote)
        self.sock.settimeout(None)        
        print("Client connected ...")

    def get_info(self):
        """retvalsurns a unique robot identifier string.
        """
        command = 0
        send_msg = pack('>H', command)
        self.sock.send(send_msg)
        time.sleep(self._delay)
        receive_msg = self.sock.recv(4096)
        retvals = unpack_from('>H', receive_msg)
        ack = retvals[0]
        if ack != ABBClient.SERVER_OK:
            raise ABBClient.CommandFailed            
        info = receive_msg[calcsize('>H'):].decode()
        return info 

    def move_joints(self, joint_angles):
        """Executes an immediate move to the specified joint angles.
        
        joint_angles = (j0, j1, j2, j3, j4, j5)
        j0, j1, j2, j3, j4, j5 are numbered from base to end effector and are
        measured in degrees (default)
        """
        joint_angles = np.array(joint_angles, dtype=np.float32).ravel()
        joint_angles *= self._scale_angle

        command = 1
        send_msg = pack('>Hffffff', command, *joint_angles)
        self.sock.send(send_msg)
        time.sleep(self._delay)
        receive_msg = self.sock.recv(4096)
        retvals = unpack_from('>H', receive_msg)
        ack = retvals[0]
        if ack != ABBClient.SERVER_OK:
            raise ABBClient.CommandFailed  

    def move_linear(self, pose):
        """Executes a linear/cartesian move from the current base frame pose to
        the specified pose.
        
        pose = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Cartesian position (default mm)
        qw, qx, qy, qz specify a quaternion rotation
        """
        pose = np.array(pose, dtype=np.float32).ravel()
        pose[:3] *= self._scale_linear
        
        command = 2
        send_msg = pack('>Hfffffff', command, *pose)
        self.sock.send(send_msg)
        time.sleep(self._delay)
        receive_msg = self.sock.recv(4096)
        retvals = unpack_from('>H', receive_msg)
        ack = retvals[0]
        if ack != ABBClient.SERVER_OK:
            raise ABBClient.CommandFailed       

    def move_circular(self, via_pose, end_pose):
        """Executes a movement in a circular path from the current base frame
        pose, through via_pose, to end_pose.
        
        via_pose, end_pose = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Cartesian position (default mm)
        qw, qx, qy, qz specify a quaternion rotation
        """        
        via_pose = np.array(via_pose, dtype=np.float32).ravel()
        via_pose[:3] *= self._scale_linear
        end_pose = np.array(end_pose, dtype=np.float32).ravel()
        end_pose[:3] *= self._scale_linear        
        
        command = 3
        send_msg = pack('>Hffffffffffffff', command, *via_pose, *end_pose)
        self.sock.send(send_msg)
        time.sleep(self._delay)
        receive_msg = self.sock.recv(4096)
        retvals = unpack_from('>H', receive_msg)
        ack = retvals[0]
        if ack != ABBClient.SERVER_OK:
            raise ABBClient.CommandFailed  

    def set_tcp(self, tcp):
        """Sets the tool center point (TCP) of the robot.
        
        The TCP is specified in the output flange frame, which is located at
        the intersection of the tool flange center axis and the flange face,
        with the z-axis aligned with the tool flange center axis.
        
        tcp = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Cartesian position (default mm)
        qw, qx, qy, qz specify a quaternion rotation
        """
        tcp = np.array(tcp, dtype=np.float32).ravel()
        tcp[:3] *= self._scale_linear
        
        command = 4
        send_msg = pack('>Hfffffff', command, *tcp)
        self.sock.send(send_msg)
        time.sleep(self._delay)
        receive_msg = self.sock.recv(4096)
        retvals = unpack_from('>H', receive_msg)
        ack = retvals[0]
        if ack != ABBClient.SERVER_OK:
            raise ABBClient.CommandFailed  

    def set_work_object(self, work_object):
        """Sets the work object on the robot.
        
        The work object is a local coordinate frame on the robot, where
        subsequent linear moves will be in this coordinate frame. 
        
        work_object = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Cartesian position (default mm)
        qw, qx, qy, qz specify a quaternion rotation
        """
        work_object = np.array(work_object, dtype=np.float32).ravel()
        work_object[:3] *= self._scale_linear
        
        command = 5
        send_msg = pack('>Hfffffff', command, *work_object)
        self.sock.send(send_msg)
        time.sleep(self._delay)
        receive_msg = self.sock.recv(4096)
        retvals = unpack_from('>H', receive_msg)
        ack = retvals[0]
        if ack != ABBClient.SERVER_OK:
            raise ABBClient.CommandFailed       

    def set_speed(self, linear_speed, angular_speed):
        """Sets the linear speed (default mm/s) and angular speed
        (default deg/s) of the robot TCP.
        """
        linear_speed *= self._scale_linear
        angular_speed *= self._scale_angle

        command = 6
        send_msg = pack('>Hff', command, linear_speed, angular_speed)
        self.sock.send(send_msg)
        time.sleep(self._delay)
        receive_msg = self.sock.recv(4096)
        retvals = unpack_from('>H', receive_msg)
        ack = retvals[0]
        if ack != ABBClient.SERVER_OK:
            raise ABBClient.CommandFailed     

    def set_zone(self, 
                 zone_key = 'z0', 
                 point_motion = False, 
                 manual_zone = None):
        zone_dict = {'z0':  (0.3, 0.3, 0.03), 
                     'z1':  (1, 1, 0.1), 
                     'z5':  (5, 8, 0.8), 
                     'z10': (10, 15, 1.5), 
                     'z15': (15, 23, 2.3), 
                     'z20': (20, 30, 3), 
                     'z30': (30, 45, 4.5), 
                     'z50': (50, 75, 7.5), 
                     'z100': (100, 150, 15), 
                     'z200': (200, 300, 30),
                    }
        """Sets the motion zone of the robot. This can also be thought of as
        the flyby zone, AKA if the robot is going from point A -> B -> C,
        how close do we have to pass by B to get to C
        
        zone_key: uses values from RAPID handbook (stored here in zone_dict)
        with keys 'z*', you should probably use these

        point_motion: go to point exactly, and stop briefly before moving on

        manual_zone = [pzone_tcp, pzone_ori, zone_ori]
        pzone_tcp: default mm, radius from goal where robot tool centerpoint 
                   is not rigidly constrained
        pzone_ori: default mm, radius from goal where robot tool orientation 
                   is not rigidly constrained
        zone_ori: default degrees, zone size for the tool reorientation
        """
        if point_motion: 
            zone = np.array((0, 0, 0))
        elif manual_zone is not None and len(manual_zone) == 3:
            zone = np.array(manual_zone, dtype=np.float32).ravel()
        elif zone_key in zone_dict.keys(): 
            zone = np.array(zone_dict[zone_key])
        else:
            raise ABBClient.InvalidZone
 
        zone[0] *= self._scale_linear
        zone[1] *= self._scale_linear
        zone[2] *= self._scale_angle

        command = 7
        send_msg = pack('>HHfff', command, int(point_motion), *zone)
        self.sock.send(send_msg)
        time.sleep(self._delay)
        receive_msg = self.sock.recv(4096)
        retvals = unpack_from('>H', receive_msg)
        ack = retvals[0]
        if ack != ABBClient.SERVER_OK:
            raise ABBClient.CommandFailed  

    def get_joint_angles(self):
        """retvalsurns the robot joint angles.
        
        joint_angles = (j0, j1, j2, j3, j4, j5)
        j0, j1, j2, j3, j4, j5 are numbered from base to end effector and are
        measured in degrees (default)
        """       
        command = 8
        send_msg = pack('>H', command)
        self.sock.send(send_msg)
        time.sleep(self._delay)
        receive_msg = self.sock.recv(4096)
        retvals = unpack_from('>Hffffff', receive_msg)
        ack = retvals[0]
        if ack != ABBClient.SERVER_OK:
            raise ABBClient.CommandFailed  
        joint_angles = np.array(retvals[1:], dtype=np.float64)
        joint_angles /= self._scale_angle
        return joint_angles
    
    def get_pose(self):
        """retvalsurns the TCP pose in the reference coordinate frame.
        
        pose = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Cartesian position (default mm)
        qw, qx, qy, qz specify a quaternion rotation
        """
        command = 9
        send_msg = pack('>H', command)
        self.sock.send(send_msg)
        time.sleep(self._delay)
        receive_msg = self.sock.recv(4096)
        retvals = unpack_from('>Hfffffff', receive_msg)
        ack = retvals[0]
        if ack != ABBClient.SERVER_OK:
            raise ABBClient.CommandFailed  
        pose = np.array(retvals[1:], dtype=np.float64)
        pose[:3] /= self._scale_linear      
        return pose
        
    def close(self):
        """Releases any resources held by the controller (e.g., sockets).
        """
        command = 99
        send_msg = pack('>H', command)
        self.sock.send(send_msg)
        time.sleep(self._delay)        
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()
        print("Shutting down client ...")
