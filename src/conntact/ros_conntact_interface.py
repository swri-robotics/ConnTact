# Copyright 2021 Southwest Research Institute
# Licensed under the Apache License, Version 2.0

# Imports for ros
import rospy
from geometry_msgs.msg import WrenchStamped, Wrench, TransformStamped, PoseStamped, Pose, Point, Quaternion, Vector3, Transform
import tf2_ros
from std_msgs.msg import String
import tf2_geometry_msgs
import tf.transformations as trfm

import numpy as np
from colorama import Fore, Back, Style

from conntact_interface import ConntactInterface

class ConntactROSInterface(ConntactInterface):
    def __init__(self, ROS_rate=100):
        self._wrench_pub = rospy.Publisher('/cartesian_compliance_controller/target_wrench', WrenchStamped,
                                           queue_size=10)
        self._pose_pub = rospy.Publisher('cartesian_compliance_controller/target_frame', PoseStamped, queue_size=2)
        self._adj_wrench_pub = rospy.Publisher('adjusted_wrench_force', WrenchStamped, queue_size=2)

        # for plotting node
        self.avg_wrench_pub = rospy.Publisher("/assembly_tools/avg_wrench", Wrench, queue_size=5)
        self.avg_speed_pub = rospy.Publisher("/assembly_tools/avg_speed", Point, queue_size=5)
        self.rel_position_pub = rospy.Publisher("/assembly_tools/rel_position", Point, queue_size=5)

        self.status_pub = rospy.Publisher("/assembly_tools/status", String, queue_size=5)

        self._ft_sensor_sub = rospy.Subscriber("/cartesian_compliance_controller/ft_sensor_wrench/", WrenchStamped,
                                               self.callback_update_wrench, queue_size=2)
        # self._tcp_pub   = rospy.Publisher('target_hole_position', PoseStamped, queue_size=2, latch=True)

        # Needed to get current pose of the robot
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        self._rate_selected = ROS_rate
        self._rate = rospy.Rate(self._rate_selected)  # setup for sleeping in hz
        self._start_time = rospy.get_rostime()
        self.curr_time = rospy.Time(0)
        self.curr_time_numpy = np.double(self.curr_time.to_sec())



    def callback_update_wrench(self, data: WrenchStamped):
        """Callback to update current wrench data whenever new data becomes available.
        """
        self.current_wrench = data


    def get_transform(self, frame, origin):
        ''' Returns the position of `end` frame relative to `start` frame.
        :param end: (string) name of target frame
        :param start: (string) name of origin frame
        :return: (geometry_msgs.TransformStamped)
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        '''
        return self.tf_buffer.lookup_transform(origin, frame, rospy.Time.now(), rospy.Duration(1))


    def get_transform_change_over_time(self, frame, origin, delta_time):
        ''' Returns the change in position of `end` frame relative to `start` frame over the last delta_time period. If the delta_time param matches this loop's frequency, the returned value will be equivalent to the instantaneous velocity.
        :param end: (string) name of target frame
        :param start: (string) name of origin frame
        :return: (geometry_msgs.TransformStamped)
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        '''

        try:
            earlierPosition = self.tf_buffer.lookup_transform(
                origin, frame,
                rospy.Time.now() - rospy.Duration(delta_time),
                rospy.Duration(2.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return (None, None)
        # Speed Diff: distance moved / time between poses
        positionDiff = self.as_array(self.current_pose.transform.translation) - self.as_array(
            earlierPosition.transform.translation)
        timeDiff = ((self.current_pose.header.stamp) - (earlierPosition.header.stamp)).to_sec()
        return (positionDiff, timeDiff)


    def register_frames(self, framesList):
        ''' Adds one or more frames of reference to the environment.
        :param framesList: (List) List of `geometry_msgs.msg.TransformStamped` frames to be added to the environment for later reference with get_pose
        '''
        self.broadcaster.sendTransform(framesList)

    def send_error(self, message, delay=0.0):
        '''Displays an error message for the user.
        :param message: (str) to display
        :param delay: (float) This particular message will not display again for this long.
        '''
        rospy.logerr_throttle(delay, message)

    def send_info(self, message, delay=0.0):
        '''Displays an error message for the user.
        :param message: (str) to display
        :param delay: (float) This particular message will not display again for this long.
        '''
        rospy.loginfo_throttle(delay, message)

    def publish_command_wrench(self, wrench: WrenchStamped):
        '''Returns a force and torque command out of Conntact and into the calling environment so that the robot can act upon that command.
        :param wrench: (WrenchStamped) commanded force and torque object.
        '''
        self._wrench_pub.publish(wrench)
        return

    def publish_averaged_wrench(self, wrench: Wrench):
        self._adj_wrench_pub.publish(wrench)
        return

    def publish_command_position(self, pose: PoseStamped):
        '''Returns a position command out of Conntact and into the calling environment so that the robot can act upon that command.
        :param pos: (PoseStamped) commanded pose object.
        '''
        self._pose_pub.publish(pose)

    def print_not_found_error(self):
        '''Whine about the abstract method not being overridden in the implementation.
        '''
        print("Abstract Conntact method {} not yet implemented.".format(inspect.stack()[1][3]))
        raise NotImplementedError()
