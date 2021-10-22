# -*- coding: utf-8 -*-
"""Dummy robot/controller classes for testing/debugging robot jogger.
"""

import logging

import numpy as np

logger = logging.getLogger(__name__)


class DummyABBController:
    """Dummy ABB controller class.
    """
    def __init__(self, ip='192.168.125.1', port=5000):
        logger.debug("DummyABBController.__init__(ip={}, port={})".format(
                ip, port))

    def __repr__(self):      
        return self.__class__.__name__

    def __str__(self):
        return self.__repr__()


class DummyRTDEController:
    """Dummy UR RTDE controller class.
    """
    def __init__(self, ip='192.168.125.1'):
        logger.debug("DummyRTDEController.__init__(ip={})".format(ip))

    def __repr__(self):      
        return self.__class__.__name__

    def __str__(self):
        return self.__repr__()


class DummyFrankxController:
    """Dummy Frankx controller class.
    """
    def __init__(self, ip='172.16.0.2'):
        logger.debug("DummyFrankxController.__init__(ip={})".format(ip))

    def __repr__(self):
        return self.__class__.__name__

    def __str__(self):
        return self.__repr__()

class DummyPyfrankaController:
    """Dummy Pyfranka controller class.
    """
    def __init__(self, ip='172.16.0.2'):
        logger.debug("DummyPyfrankaController.__init__(ip={})".format(ip))

    def __repr__(self):
        return self.__class__.__name__

    def __str__(self):
        return self.__repr__()

class DummySyncRobot:
    """Dummy synchronous robot class.
    """

    EULER_AXES = ("sxyz", "sxyx", "sxzy", "sxzx", "syzx", "syzy",
                  "syxz", "syxy", "szxy", "szxz", "szyx", "szyz",
                  "rzyx", "rxyx", "ryzx", "rxzx", "rxzy", "ryzy",
                  "rzxy", "ryxy", "ryxz", "rzxz", "rxyz", "rzyz",
                  )
    
    def __init__(self, ctrl):
        self.ctrl = ctrl
        logger.debug("DummySyncRobot.__init__(ctrl={})".format(ctrl))

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
        return "Dummy robot info"

    @property
    def axes(self):
        return 'rxyz'
    
    @axes.setter
    def axes(self, axes):
        logger.debug("DummySyncRobot.axes.setter(axes={})".format(axes))

    @property
    def tcp(self):
        return np.array((0.12345, 0.0, 89.1, 0.0, 0.0, 0.0))

    @tcp.setter
    def tcp(self, tcp):
        logger.debug("DummySyncRobot.tcp.setter(tcp={})".format(tcp))

    @property
    def coord_frame(self):
        return np.array((300.12345, 400.0, 300.0, 180.0, 0.0, 180.0))

    @coord_frame.setter
    def coord_frame(self, frame):
        logger.debug("DummySyncRobot.coord_frame.setter(frame={})".format(
                frame))

    @property
    def linear_speed(self):
        return 20.12345

    @linear_speed.setter
    def linear_speed(self, speed):
        logger.debug("DummySyncRobot.linear_speed.setter(speed={})".format(
                speed))

    @property
    def angular_speed(self):
        return 10.12345

    @angular_speed.setter
    def angular_speed(self, speed):
        logger.debug("DummySyncRobot.angular_speed.setter(speed={})".format(
                speed))

    @property
    def blend_radius(self):
        return 5.12345

    @blend_radius.setter
    def blend_radius(self, blend_radius):
        logger.debug("DummySyncRobot.blend_radius.setter(radius={})".format(
                blend_radius))

    @property
    def joint_angles(self):
        if isinstance(self.ctrl, DummyFrankxController):
            # Franka arm has 7 joints
            return np.array((0.12345, 10.0, 20.0, 30.0, 40.0, 50.0, 100.0))
        else:
            # ABB and UR5 arms have 6 joints
            return np.array((0.12345, 10.0, 20.0, 30.0, 40.0, 50.0))

    @property
    def pose(self):
        return np.array((50.12345, 60.0, 70.0, -10.0, 20.0, -30.0))

    def move_joints(self, joint_angles):
        logger.debug("DummySyncRobot.move_joints(angles={})".format(
                joint_angles))
    
    def move_linear(self, pose):
        logger.debug("DummySyncRobot.move_linear(pose={})".format(
                pose))

    def move_circular(self, via_pose, end_pose):
        logger.debug("DummySyncRobot.move_circular(via pose={}, end pose={})".format(
                via_pose, end_pose))        
        
    def close(self):
        logger.debug("DummySyncRobot.close")


# Uncomment for offline debugging
# SyncRobot = DummySyncRobot
# ABBController = DummyABBController
# RTDEController = DummyRTDEController
# FrankxController = DummyFrankxController