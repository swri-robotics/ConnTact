# Copyright 2021 Southwest Research Institute
# Licensed under the Apache License, Version 2.0

import numpy as np
from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion, Transform,
                               TransformStamped, Vector3, Wrench,
                               WrenchStamped)
import tf.transformations as tf

def qToEu(a):
    """
    :param a: (np.ndarray) Quaternion of the form (x,y,z,w)
    :return: (np.ndarray) Euler angles in degrees (XYZ sequential)
    """
    return np.degrees(tf.euler_from_quaternion([*a]))

def euToQ(a):
    """
    :param a: (np.ndarray) Euler angles in degrees (XYZ sequential)
    :return: (np.ndarray) Quaternion of the form (x,y,z,w)
    """
    return tf.quaternion_from_euler(*np.radians(a))

def qGetMagnitude(a):
    """
    Get the magnitude of a quaternion (in degrees) based on its w component.
    :param a: (np.ndarray) Quaternion of the form (x,y,z,w)
    :return: (float) Angle change in degrees.
    """
    return np.degrees(np.arccos(a[3])*2)

def quat_lerp(q1, q2, factor):
    """
    Return a quasi-linear interpolation between q1 and q2. To get a direct motion across the
    surface of a sphere, we actually convert to euler angles and interpolate in that space.
    :param q1: (np.ndarray) Quaternion of the form (x,y,z,w) specifying base orientation
    :param q2: (np.ndarray) Quaternion of the form (x,y,z,w) specifying target orientation
    :return: (float) Interpolation fraction (0.0-1.0) of the way from base to target.
    :return: (np.ndarray) Quaternion of the form (x,y,z,w)
    """
    return euToQ(factor * qToEu(q2) + (1-factor) * qToEu(q1))

def euler_lerp(e1, e2, factor):
    """
    Return a quasi-linear interpolation between euler rotations e1 and e2. To get direct motion
    in the robot, interpolate axially like this, not slerping quaternions.
    :param q1: (np.ndarray) Euler rotation of the form (x,y,z) sequential specifying base orientation
    :param q2: (np.ndarray) Euler rotation of the form (x,y,z) specifying target orientation
    :return: (float) Interpolation fraction (0.0-1.0) of the way from base to target.
    :return: (np.ndarray) Euler rotation of the form (x,y,z)
    """
    return (factor * e2 + (1-factor) * e1)

def euGetMagnitude(euler):
    return np.linalg.norm(euler)

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

def interpCommandByMagnitude(curr_vec,target_vec, lead_maximum=[.1,1]):
    """
    Shorten a command's 'lead' to a given pos/rot cap to artificially restrict motion speed on a PD controller.
    We take in the current position and the initial target position, and return a modified target position
    which can't be further than the bounds specified.
    :param curr_vec: (list) = [[x,y,z position],[rotation in either Euler or Quaternion]]
    """
    # print("Current pose:{}".format(curr_vec))
    # Keep track of whether a command change was required. If the target lead is small, there's no need.
    changed = False

    # Record whether a quaternion was passed in instead of an Euler rotation:
    input_quaternion = len(curr_vec[1]) == 4
    assert len(curr_vec[1])==len(target_vec[1]), "Different orientation representations sent to interp!"
    if input_quaternion: #We're getting a quaternion for orientation; change it to Euler for the process
        curr_vec[1] = qToEu(curr_vec[1])
        target_vec[1] = qToEu(target_vec[1])

    # Get magnitude of move and rotation by taking the difference from current to target:
    diff_vec = [target_vec[0]-curr_vec[0],
                target_vec[1]-curr_vec[1]]
    trans_mag   = np.linalg.norm(diff_vec[0])
    rot_mag     = euGetMagnitude(diff_vec[1])
    new_command_vec = [*target_vec]

    # Clip magnitudes
    if trans_mag > lead_maximum[0]:
        new_command_vec[0] = (lead_maximum[0] / trans_mag) * np.array(diff_vec[0]) + curr_vec[0]
        trans_mag = np.linalg.norm(new_command_vec[0])
        changed = True
    if rot_mag > lead_maximum[1]:
        new_command_vec[1] = euler_lerp(curr_vec[1], target_vec[1], (lead_maximum[1] / rot_mag))
        rot_mag = euGetMagnitude(new_command_vec[1])
        changed = True

    if not changed:
        # convert command back to quaternion if we started with one:
        if input_quaternion:
            target_vec[1] = euToQ(target_vec[1])
        # print("Clipper returning unchanged target {}.".format(target_vec))
        # print("Clipped command gives dist {} and rot {}".format(trans_mag, rot_mag))
        return target_vec #, [trans_mag, rot_mag], False


    # Add the commanded lead back to
    if changed:    # convert command back to quaternion if we started with one:
        if input_quaternion:
            new_command_vec[1] = euToQ(new_command_vec[1])
        # print("Clipper returning changed target: \n{}".format(new_command_vec))
        return new_command_vec #,[trans_mag,rot_mag], True

# print("Yay")
class AssemblyFilters():
    """Averages a signal based on a history log of previous values. Window size is normallized to different
    frequency values using _rate_selected; window should be for 100hz cycle time.
    """

    def __init__(self, window=15, rate_selected=100):

        # Simple Moving Average Parameters
        self._rate_selected = rate_selected
        self._buffer_window = dict()
        self._buffer_window[
            "wrench"] = window  # should tie to self._rate_selected = 1/Hz since this variable is the rate of ROS commands
        self._data_buffer = dict()


    def average_wrench(self, input) -> np.ndarray:
        # out = input
        # Combine
        force = self.average_threes(input.force, 'force')
        torque = self.average_threes(input.torque, 'torque')

        return Wrench(self.dict_to_point(force), self.dict_to_point(torque))

    def average_speed(self, input) -> np.ndarray:
        """Takes speed as a list of components, returns smoothed version
        :param input: (numpy.Array) Speed vector
        :return: (numpy.Array) Smoothed speed vector
        """

        speed = self.average_threes(Point(input[0], input[1], input[2]), 'speed')
        return np.array([speed['x'], speed['y'], speed['z']])

    def average_threes(self, input, name):
        """Returns the moving average of a dict of x,y,z values
        :param input: (geometry_msgs.msg.Point) A point with x,y,z properties
        :param name: (string) Name to use for buffer dictionary
        :return: (dict) x,y,z dictionary of the averaged values.
        """

        vals = self.point_to_dict(input)
        for k, v in vals.items():
            vals[k] = self.simple_moving_average(v, 15, key=name + '_' + k)
        return vals

    def point_to_dict(self, input):
        return {"x": input.x, "y": input.y, "z": input.z}

    def dict_to_point(self, input):
        return Point(input["x"], input["y"], input["z"])

    def simple_moving_average(self, new_data_point, window=None, key="wrench"):

        if not key in self._data_buffer:
            self._data_buffer[key] = np.array([])
            self._buffer_window[key] = window
        window = int(np.floor(
            self._buffer_window[key] * self._rate_selected / 100))  # Unless new input provided, use class member
        # Fill up the first window while returning current value, else calculate moving average using constant window
        if len(self._data_buffer[key]) < window:
            self._data_buffer[key] = np.append(self._data_buffer[key], new_data_point)
            avg = self.calc_moving_average(self._data_buffer[key], len(self._data_buffer[key]))
        else:
            self._data_buffer[key] = np.append(self._data_buffer[key],
                                               new_data_point)  # append new datapoint to the end
            self._data_buffer[key] = np.delete(self._data_buffer[key], 0)  # pop the first element
            avg = self.calc_moving_average(self._data_buffer[key], window)

        return avg

    def calc_moving_average(self, buffered_data, w):  # w is the window
        return np.convolve(buffered_data, np.ones(w), 'valid') / w

