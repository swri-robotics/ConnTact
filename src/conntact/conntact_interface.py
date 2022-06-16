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
        ''' Returns the position of `end` frame relative to `start` frame.
        :param end: (string) name of target frame
        :param start: (string) name of origin frame
        :return: (geometry_msgs.TransformStamped)
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        '''
        self.printNotFoundError()
        pass

    @abc.abstractmethod
    def register_frames(self, framesList):
        ''' Adds one or more frames of reference to the environment.
        :param framesList: (List) List of `geometry_msgs.msg.TransformStamped` frames to be added to the environment for later reference with get_pose
        '''
        self.printNotFoundError()
        pass

    @abc.abstractmethod
    def get_transform_change_over_time(self, frame, origin, delta_time):
        ''' Returns the change in position of `end` frame relative to `start` frame over the last delta_time period. If the delta_time param matches this loop's frequency, the returned value will be equivalent to the instantaneous velocity.
        :param end: (string) name of target frame
        :param start: (string) name of origin frame
        :return: (geometry_msgs.TransformStamped)
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        '''
        self.printNotFoundError()
        pass

    def printNotFoundError(self):
        '''Whine about the abstract method not being overridden in the implementation.
        '''
        print("Abstract Conntact method {} not yet implemented.".format(inspect.stack()[1][3]))
        raise NotImplementedError()


class ConntactROSInterface(ConntactInterface):
    def __init__(self, ROS_rate = 100):
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


        self._rate_selected         = ROS_rate
        self._rate                  = rospy.Rate(self._rate_selected) #setup for sleeping in hz
        self._start_time            = rospy.get_rostime()
        self.curr_time              = rospy.Time(0)
        self.curr_time_numpy        = np.double(self.curr_time.to_sec())


    def get_transform(self, frame, origin):
        return self.tf_buffer.lookup_transform('target_hole_position', 'tool0', rospy.Time(0), rospy.Duration(1.25))

    def get_transform_change_over_time(self, frame, origin, delta_time):
        """Updates a simple moving average of robot tcp speed in mm/s. A speed is calculated from the difference between a
          previous pose (.1 s in the past) and the current pose; this speed is filtered and stored as self.average_speed.
         """
        curr_time = rospy.get_rostime() - self._start_time
        if (curr_time.to_sec() > rospy.Duration(.5).to_sec()):
            try:
                earlierPosition = self.tf_buffer.lookup_transform("base_link",
                                                                  self.tool_data[self.activeTCP]['transform'].child_frame_id,
                                                                  rospy.Time.now() - rospy.Duration(.1),
                                                                  rospy.Duration(2.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                raise
            # Speed Diff: distance moved / time between poses
            positionDiff = self.as_array(self.current_pose.transform.translation) - self.as_array(
                earlierPosition.transform.translation)
            timeDiff = ((self.current_pose.header.stamp) - (earlierPosition.header.stamp)).to_sec()
            if (timeDiff > 0.0):  # Update only if we're using a new pose; also, avoid divide by zero
                speedDiff = positionDiff / timeDiff
                # Moving averate weighted toward old speed; response is independent of rate selected.
                # self.average_speed = self.average_speed * (1-10/self._rate_selected) + speedDiff * (10/self._rate_selected)
                # rospy.logwarn_throttle(2.0, "Speed is currently about " + str(speedDiff))
                self.average_speed = self.filters.average_speed(speedDiff)
        else:
            rospy.logwarn_throttle(1.0, "Too early to report past time!" + str(curr_time.to_sec()))

