#! /usr/bin/env python3

# Copyright 2021 Southwest Research Institute
# Licensed under the Apache License, Version 2.0
import inspect
import abc


# for the ros interface, to be extracted to a new module:
import numpy as np
import rospy
import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped, Wrench, TransformStamped, PoseStamped, Pose, Point, Quaternion, Vector3, Transform

class ConntactInterface():


    @abc.abstractmethod
    def get_transform(self, frame, origin):
        """ Returns the position of `end` frame relative to `start` frame.
        :param frame: (string) name of target frame
        :param origin: (string) name of origin frame
        :return: (geometry_msgs.TransformStamped)
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        """
        self.print_not_found_error()
        pass

    @abc.abstractmethod
    def register_frames(self, framesList):
        """ Adds one or more frames of reference to the environment.
        :param framesList: (List) List of `geometry_msgs.msg.TransformStamped` frames to be added to the environment for later reference with get_pose
        """
        self.print_not_found_error()
        pass

    @abc.abstractmethod
    def get_transform_change_over_time(self, frame, origin, delta_time):
        """ Returns the change in position of `end` frame relative to `start` frame over the last delta_time period. If the delta_time param matches this loop's frequency, the returned value will be equivalent to the instantaneous velocity.
        :param frame: (string) name of target frame
        :param origin: (string) name of origin frame
        :return: (geometry_msgs.TransformStamped)
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        """
        self.print_not_found_error()
        pass

    @abc.abstractmethod
    def send_error(self, message, delay = 0.0):
        """Displays an error message for the user.
        :param message: (str) to display
        :param delay: (float) This particular message will not display again for this long.
        """
        self.print_not_found_error()
        pass

    @abc.abstractmethod
    def send_info(self, message, delay = 0.0):
        """Displays an error message for the user.
        :param message: (str) to display
        :param delay: (float) This particular message will not display again for this long.
        """
        self.print_not_found_error()
        pass

    @abc.abstractmethod
    def publish_command_wrench(self, wrench:WrenchStamped):
        """Returns a force and torque command out of Conntact and into the calling environment so that the robot can act upon that command.
        :param wrench: (WrenchStamped) commanded force and torque object.
        """
        self.print_not_found_error()
        pass

    @abc.abstractmethod
    def publish_command_position(self, pose:PoseStamped):
        """
        Returns a position command out of Conntact and into the calling environment so that the robot can act upon that command.
        :param pos: (PoseStamped) commanded pose object.
        """
        self.print_not_found_error()
        pass

    @abc.abstractmethod
    def publish_averaged_wrench(self, wrench: Wrench):
        """
        Refactor of graphing solution should remove this. Publish the averaged-out wrench sensor readings for graphing.
        """
        self.print_not_found_error()
        pass

    @abc.abstractmethod
    def publish_plotting_values(self, items: dict):
        self.print_not_found_error()
        pass

    @abc.abstractmethod
    def sleep_until_next_loop(self):
        self.print_not_found_error()
        pass

    @abc.abstractmethod
    def print_not_found_error(self):
        """
        Whine about the abstract method not being overridden in the implementation.
        """
        print("Abstract Conntact method {} not yet implemented.".format(inspect.stack()[1][3]))
        raise NotImplementedError()

