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
    if type(a) is Quaternion:
        return qToEu([a.x, a.y, a.z, a.w])
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
    return np.degrees(np.arccos(a[3]) * 2)


def quat_lerp(q1, q2, factor):
    """
    Return a quasi-linear interpolation between q1 and q2. To get a direct motion across the
    surface of a sphere, we actually convert to euler angles and interpolate in that space.
    :param q1: (np.ndarray) Quaternion of the form (x,y,z,w) specifying base orientation
    :param q2: (np.ndarray) Quaternion of the form (x,y,z,w) specifying target orientation
    :return: (float) Interpolation fraction (0.0-1.0) of the way from base to target.
    :return: (np.ndarray) Quaternion of the form (x,y,z,w)
    """
    return euToQ(factor * qToEu(q2) + (1 - factor) * qToEu(q1))


def euler_lerp(e1, e2, factor):
    """
    Return a quasi-linear interpolation between euler rotations e1 and e2. To get direct motion
    in the robot, interpolate axially like this, not slerping quaternions.
    :param q1: (np.ndarray) Euler rotation of the form (x,y,z) sequential specifying base orientation
    :param q2: (np.ndarray) Euler rotation of the form (x,y,z) specifying target orientation
    :return: (float) Interpolation fraction (0.0-1.0) of the way from base to target.
    :return: (np.ndarray) Euler rotation of the form (x,y,z)
    """
    return (factor * e2 + (1 - factor) * e1)


def euGetMagnitude(euler):
    return np.linalg.norm(euler)


class MoveMode:
    """
    Base class for MoveModes
    """

    def get_move(self, current_position: np.ndarray,
                 vector: np.ndarray,
                 origin: np.ndarray) -> np.ndarray:
        """
        Performs the projection of current position according to MoveMode.
        """
        return


class MoveModeSet(MoveMode):
    def get_move(self, current_position: np.ndarray,
                 vector: np.ndarray,
                 origin: np.ndarray) -> np.ndarray:
        """
        Always returns Origin only, sending the robot to the commanded position.
        """
        return origin


class MoveModeLine(MoveMode):
    def project_on_line(self, current_pos: np.ndarray,
                        vector: np.ndarray,
                        origin: np.ndarray) -> np.ndarray:
        """
        Get the vector from origin to current_position;
        subtract away the component parallel to vector;
        add position of origin back.
        """
        diff = current_pos - origin
        # Project onto vector:
        projected_length = np.dot(diff, vector) / np.linalg.norm(vector)
        return origin + (vector * projected_length / np.linalg.norm(vector))

    def get_move(self, current_position: np.ndarray,
                 vector: np.ndarray,
                 origin: np.ndarray) -> np.ndarray:
        """
        Always returns Origin only, sending the robot to the commanded position.
        """
        return self.project_on_line(current_position, vector, origin)


class MoveModePlane(MoveMode):
    def project_on_plane(self, current_pos: np.ndarray,
                         vector: np.ndarray,
                         origin: np.ndarray) -> np.ndarray:
        """
        Get the vector from origin to current_position;
        subtract away the component parallel to vector;
        add position of origin back.
        """
        diff = current_pos - origin
        # Project onto vector:
        projected_length = np.dot(diff, vector) / np.linalg.norm(vector)
        return origin + (diff - vector * projected_length / np.linalg.norm(vector))

    def get_move(self, current_position: np.ndarray,
                 vector: np.ndarray,
                 origin: np.ndarray) -> np.ndarray:
        """
        Always returns Origin only, sending the robot to the commanded position.
        """
        return self.project_on_plane(current_position, vector, origin)


class MoveModeFree(MoveMode):
    """Doesn't constrain robot motion at all."""

    def get_move(self, current_position: np.ndarray,
                 vector: np.ndarray,
                 origin: np.ndarray) -> np.ndarray:
        """
        Always returns Origin only, sending the robot to the commanded position.
        """
        return current_position


class MovePolicy:
    """
    Policy for selective compliance along arbitrary axes.
    MoveMode:
    :param MoveMode:  (np.array(3)) Select the movement mode from the list,
        and a policy object will be created to maintian that policy.
        "Set": The robot will attempt to move to the exact coordinates passed as Origin.
        "Line": The robot will move freely along a vector in target space. Vector's origin
            robot TCP position when the MoveMode is selected; to move the start point, pass
            in optional command "start_point".
        "Plane": As "line" MoveMode, except the robot will hold to the plane normal to the
            vector and passing through the origin point.
        "Free": The robot moves freely in all directions.
    :param vector: (np.array(3)) In Line or Plane MoveMode, the vector onto/about which the robot's
        current position is projected to generate the commanded position.
    :param origin: (np.array(3)) In the Set MoveMoveMode, the point the robot will try to attain. In Line
        or Plane MoveMode, the origin point of the vector. When you set/change the MoveMode, this point
        updates to the robot TCP's current position by default, unless you pass a value in.
    :param orientation: (np.array(3)) Euler angles representation of the desired orientation. TODO: Make dynamic?
    :param force_cmd:  (list of floats) Force to apply at the TCP. Moves the robot through space
        according to MoveMode. An equal and opposite force from the physical sensor will stop this motion.
    :param torque_cmd:  (list of floats) XYZ torques to apply relative to target.
    :return: ([np.array(3),np.array(3)]) Rotation and orientation of command pose.
    """

    # MovePolicy init
    def __init__(self, 
                orientation,
                 move_mode: str = None,
                 vector = None,
                 origin = None,
                 force_cmd = [0, 0, 0],
                 torque_cmd = [0, 0, 0]):
        # Initialize to None; None is fine for Free mode.
        self._vector = None
        self._origin = None
        self._move_mode: MoveMode = None
        # Run the getter functions if we got args:
        if vector is not None:
            self.vector = vector
        if origin is not None:
            self.origin = origin
        if move_mode is not None:
            self.move_mode: MoveMode = move_mode
        self.orientation = orientation
        self.torque = torque_cmd
        self.force = force_cmd

    @property
    def move_mode(self):
        """Create a new movemode according to the String"""
        return self._move_mode

    @move_mode.setter
    def move_mode(self, mode):
        """Create a new MoveMode according to a string."""
        if isinstance(mode, MoveMode):  # Allows passing a mode name so IDE autocorrect activates.
            self._move_mode = mode
        elif type(mode) is str:
            if mode.lower() == "set":
                self._move_mode = MoveModeSet()
            elif mode.lower() == "line":
                if self.vector is None:
                    raise TypeError("MovePolicy() Line mode requires argument 'vector'")
                self._move_mode = MoveModeLine()
            elif mode.lower() == "plane":
                if self.vector is None:
                    raise TypeError("MovePolicy() Plane mode requires argument 'vector'")
                self._move_mode = MoveModePlane()
            elif mode.lower() == "free":
                self._move_mode = MoveModeFree()
        else:
            raise TypeError("Wrong type passed to create new movemode! Requires str or MoveMode.")

    @property
    def vector(self):
        return self._vector

    @vector.setter
    def vector(self, value):
        # if value.all() != self._vector.all():
        self._vector = self.validate_vec(value)

    @property
    def origin(self):
        return self._origin

    @origin.setter
    def origin(self, value):
        self._origin = self.validate_vec(value)

    @property
    def wrench(self):
        """Return a wrench as a list, used for conversion to Wrench message object"""
        return [self.force, self.torque]

    def info(self):
        return {"move_mode":type(self.move_mode),
                "vector":self.vector,
                "origin":self.origin,
                "orientation":self.orientation,
                "force":self.force,
                "torque":self.torque
                }

    def validate_vec(self, value):
        """
        Check that the input vector is indeed a valid 3-element NP array. Always stores the vector on the
        move_policy as a 3-element np.ndarray
        :param value: (Vector3, List, or np.ndarray) set of 3 floats
        """
        # automatically translate Vector3 datatype for simplicity.
        if type(value) is Vector3:
            return self.validate_vec([value.x,
                                      value.y,
                                      value.z])
        if len(value) != 3:
            raise TypeError("Wrong number of elements in 3-d vector passed to MovePolicy! {}".format(value))
        if not all(type(thing) in (int, float, np.int64, np.float64) for thing in value):
            raise TypeError("Wrong data type passed as elements in vector for MovePolicy! "
                            "Expected float or int. Got {}".format(value))
        if type(value) == list:
            #Automatically store the value as a numpy array for math purposes.
            return np.array(value)
        return value

    def current_move(self, current_position: np.ndarray) -> np.ndarray:
        """Get the move specified by the policy at this position.
        :param current_position: current robot position in task space.
        :return: resulting commanded position in task space.
        """
        if self.origin is None:  # Origin defaults to the current TCP position if not provided
            self.origin = current_position
        if type(current_position) in (Vector3, Point):
            return self.current_move([current_position.x,
                                      current_position.y,
                                      current_position.z])
        return [self.move_mode.get_move(current_position, self.vector, self.origin),
                self.orientation]


def interp_command_by_magnitude(curr_vec, target_vec, lead_maximum=[.1, 3]):
    """
    Shorten a command's 'lead' to a given pos/rot cap to artificially restrict motion speed on a PD controller.
    We take in the current position and the initial target position, and return a modified target position
    which can't be further than the bounds specified.
    :param curr_vec: (list) = [[x,y,z position],[rotation in either Euler or Quaternion]] current pose
    :param target_vec: (list) = [[x,y,z position],[rotation in either Euler or Quaternion]] target pose
    :param lead_maximum: (list) = [float distance in meters, rotation in degrees] offset from current position toward
    target_vec to which to limit the returned interpolated command.
    :return: (list) [[x,y,z position],[rotation in either Euler or Quaternion]]
    """
    # print("Current pose:{}".format(curr_vec))
    # Keep track of whether a command change was required. If the target lead is small, there's no need.
    changed = False

    # Record whether a quaternion was passed in instead of an Euler rotation:
    input_quaternion = len(curr_vec[1]) == 4
    assert len(curr_vec[1]) == len(target_vec[1]), "Different orientation representations sent to interp!"
    if input_quaternion:  # We're getting a quaternion for orientation; change it to Euler for the process
        curr_vec[1] = qToEu(curr_vec[1])
        target_vec[1] = qToEu(target_vec[1])

    # Get magnitude of move and rotation by taking the difference from current to target:
    diff_vec = [target_vec[0] - curr_vec[0],
                target_vec[1] - curr_vec[1]]
    trans_mag = np.linalg.norm(diff_vec[0])
    rot_mag = euGetMagnitude(diff_vec[1])
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
        return target_vec  # , [trans_mag, rot_mag], False

    # Add the commanded lead back to
    if changed:  # convert command back to quaternion if we started with one:
        # if input_quaternion:
        #     new_command_vec[1] = euToQ(new_command_vec[1])
        return new_command_vec  # ,[trans_mag,rot_mag], True


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
