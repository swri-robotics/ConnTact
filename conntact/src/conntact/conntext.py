# Copyright 2021 Southwest Research Institute
# Licensed under the Apache License, Version 2.0

# Imports for ros
import time

from geometry_msgs.msg import WrenchStamped, Wrench, TransformStamped, PoseStamped, Pose, Point, Quaternion, Vector3, \
    Transform
import tf2_ros
import tf2_geometry_msgs
import inspect


import numpy as np
from colorama import Fore, Back, Style
import tf.transformations as trfm
import inspect
import yaml

from conntact.conntact_interface import ConntactInterface

from modern_robotics import Adjoint as homogeneous_to_adjoint, RpToTrans
import conntact.assembly_utils as utils

class ToolData():
    def __init__(self):
        self.name = "tool0"
        self.frame_name = "tool0"
        self.transform = None
        self.matrix = None
        self.validToolsDict = None

class Conntext:

    def __init__(self, interface: ConntactInterface, conntact_params="conntact_params"):
        self._current_pose = None
        self._current_wrench = None


        # Save a reference to the interface.
        self.interface = interface
        self.params = self.interface.load_yaml_file(conntact_params)

        # Save the refresh rate as read-only property
        self.rate = self.params["framework"]["refresh_rate"]

        # Instantiate the dictionary of frames which are always required for tasks.
        self.reference_frames = {"tcp": TransformStamped()}
        self.highForceWarning = False
        self.target_frame_name = "tool0"

        #For debugging:
        self.motion_permitted = True
        # self.motion_permitted = False

        # Initialize filtering class
        self.filters = utils.AssemblyFilters(5, self.rate)
        self.toolData = ToolData()

        # Initialize values for core properties
        self.update_current_pos()
        self._current_wrench = self.create_wrench([0, 0, 0], [0, 0, 0])
        self._average_wrench_gripper = self.create_wrench([0, 0, 0], [0, 0, 0]).wrench
        self._average_wrench_world = Wrench()
        self.average_speed = np.array([0.0,0.0,0.0])

        self.conntask = None #A reference to the Conntask. If it stops existing, we stop sending robot motion commands.


    #Establish the most important data as read-only
    @property
    def current_pose(self):
        return self._current_pose
    @property
    def current_wrench(self):
        return self._current_wrench

    def update(self):
        #All once-per-loop functions
        self.interface.send_info("Updating current pos, avg speed and wrench.", 2)
        self.update_current_pos()
        self.update_avg_speed()
        self.update_average_wrench()

    def send_reference_TFs(self):
        self.interface.register_frames(self.reference_frames)

    def set_target_frame_name(self, name):
        self.target_frame_name = name
        # self.interface.send_info("Changing target frame, watch out!")
        # time.sleep(10)

    @staticmethod
    def get_tf_from_yaml(pos, ori, base_frame,
                         child_frame):  # Returns the transform from base_frame to child_frame based on vector inputs
        """Reads a TF from config YAML.
        :param pos: (string) Param key for desired position parameter.
        :param ori: (string) Param key for desired orientation parameter.
        :param base_frame: (string) Base frame for output TF.
        :param child_frame:  (string) Child frame for output TF.
        :return: Geometry_Msgs.TransformStamped with linked parameters.
        """

        output_pose = Conntext.get_pose_from_yaml(pos, ori, base_frame)  # tf_task_board_to_hole
        output_tf = TransformStamped()
        output_tf.header = output_pose.header
        # output_tf.transform.translation = output_pose.pose.position
        [output_tf.transform.translation.x, output_tf.transform.translation.y, output_tf.transform.translation.z] = [
            output_pose.pose.position.x, output_pose.pose.position.y, output_pose.pose.position.z]
        output_tf.transform.rotation = output_pose.pose.orientation
        output_tf.child_frame_id = child_frame

        return output_tf

    @staticmethod
    def get_pose_from_yaml(pos, ori, base_frame):  # Returns the pose wrt base_frame based on vector inputs.
        """Reads a Pose from config YAML.
        :param pos: (string) Param key for desired position parameter.
        :param ori: (string) Param key for desired orientation parameter.
        :param base_frame: (string) Base frame for output pose.
        :return: Geometry_Msgs.PoseStamped with linked parameters.
        """

        # Inputs are in mm XYZ and degrees RPY
        # move to utils
        output_pose = PoseStamped()  # tf_task_board_to_hole
        # output_pose.header.stamp = self.interface.get_unified_time()
        output_pose.header.frame_id = base_frame
        tempQ = list(trfm.quaternion_from_euler(ori[0] * np.pi / 180, ori[1] * np.pi / 180, ori[2] * np.pi / 180))
        output_pose.pose = Pose(Point(pos[0] / 1000, pos[1] / 1000, pos[2] / 1000),
                                Quaternion(tempQ[0], tempQ[1], tempQ[2], tempQ[3]))
        return output_pose

    def select_tool(self, tool_name):
        """Sets activeTCP frame according to title of desired peg frame (tip, middle, etc.). This frame must be included in the YAML.
        :param tool_name: (string) Key in tool_data dictionary for desired frame.
        """
        # TODO: Make this a loop-run state to slowly slerp from one TCP to another using
        #  https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Slerp.html

        if tool_name in list(self.toolData.validToolsDict):
            self.toolData.name = tool_name
            self.toolData.frame_name = "peg_" + tool_name + "_position"
            gripper_to_tool_tip_transform = Conntext.get_tf_from_yaml(
                self.toolData.validToolsDict[tool_name]['pose'],
                self.toolData.validToolsDict[tool_name]['orientation'],
                "tool0_to_gripper_tip_link", self.toolData.frame_name)
            self.reference_frames['tcp'] = gripper_to_tool_tip_transform
            self.send_reference_TFs()
            # get the transform from tool0 (which CartesianControllers treats as the root frame) to the TCP
            self.toolData.transform = self.interface.get_transform(self.toolData.frame_name, "tool0")
            self.toolData.matrix    = Conntext.to_homogeneous(
                self.toolData.transform.transform.rotation,
                self.toolData.transform.transform.translation
                )
            print("New tool position selected: {}".format(self.toolData.name))

        else:
            self.interface.send_error("Tool selection key error! No key '" +
                                      tool_name + "' in tool dictionary.", 2)

    def update_current_pos(self):
        """Read in current pose from robot base to activeTCP.        
        """
        self._current_pose = self.interface.get_transform(
            self.toolData.frame_name,
            self.target_frame_name)

    def get_command_wrench(self, vec=[0, 0, 0], ori=[0, 0, 0]):
        """Output ROS wrench parameters from human-readable vector inputs.
        :param vec: (list of floats) Vector of desired force in each direction (in Newtons).
        :param ori: (list of floats) Vector of desired torque about each axis (in N*m)
        """
        return [vec[0], vec[1], vec[2], ori[0], ori[1], ori[2]]

    def publish_wrench(self, input_vec):
        """Publish the commanded wrench to the command topic.
        :param vec: (list of Floats) XYZ force commands
        :param vec: (list of Floats) XYC commanded torque.
        """

        result_wrench = self.create_wrench(input_vec[0], input_vec[1])

        transform_world_to_gripper: TransformStamped = self.interface.get_transform('target_hole_position', 'tool0')
        tcp_position = self.toolData.transform.transform.translation
        offset = Point(-1 * tcp_position.x, -1 * tcp_position.y, -1 * tcp_position.z - .05)

        transform_world_to_gripper.transform.translation = offset

        # Execute reinterpret-to-tcp and rotate-to-world simultaneously:
        result_wrench.wrench = Conntext.transform_wrench(transform_world_to_gripper,
                                                              result_wrench.wrench)  # This works
        if(self.motion_permitted):
            self.interface.publish_command_wrench(result_wrench)

    @staticmethod
    def list_from_quat(quat):
        return [quat.x, quat.y, quat.z, quat.w]

    @staticmethod
    def list_from_point(point):
        return [point.x, point.y, point.z]

    def limit_speed(self, pose_vec):
        """Takes in vector representations of position
        :param pose_stamped_vec: (list of floats) List of parameters for pose with x,y,z position and orientation quaternion
        """
        limit = np.array(self.params['robot']['max_pos_change_per_second'])
        curr_pos = self.as_array(self.current_pose.transform.translation)
        rot = self.current_pose.transform.rotation
        curr_ori = utils.qToEu([rot.x,
                                rot.y,
                                rot.z,
                                rot.w])

        return utils.interpCommandByMagnitude(np.array([curr_pos, [*curr_ori]]), np.array([*pose_vec]), [.05, 2])

    def publish_pose(self, pose_stamped_vec):
        """Takes in vector representations of position and rotation [[pos],[rot]] in task space and publish
        them as a pose_stamped for the robot to attain.
        :param pose_stamped_vec: (list of floats) List of parameters for pose with x,y,z position and orientation quaternion
        """
        # self.interface.send_info("Processing move command {}.".format(pose_stamped_vec), 1 )

        if (pose_stamped_vec is None):
            self.interface.send_info("Command position not initialized yet...")
            return

        # TODO: Add pose_stamped_vec to current position here.
        pose_command = self.limit_speed(pose_stamped_vec)
        # pose_command = conntact.utils.interpCommandByMagnitude(pose_stamped_vec, [.1, 20])

        # Create poseStamped msg
        goal_pose = PoseStamped()
        # Set the position and orientation
        goal_pose.pose.position = Point(*pose_command[0])
        goal_pose.pose.orientation = Quaternion(*utils.euToQ(pose_command[1]))
        # Set header values
        goal_pose.header.stamp = self.interface.get_unified_time()
        goal_pose.header.frame_id = self.target_frame_name
        # goal_pose.header.frame_id = "base_link"

        goal_pose = self.interface.do_transform(goal_pose, "base_link")

        if (self.toolData.name != "tool0"):
            # Convert pose in TCP coordinates to assign wrist "tool0" position for controller

            b_link = goal_pose.header.frame_id
            goal_matrix = Conntext.to_homogeneous(goal_pose.pose.orientation,
                                                       goal_pose.pose.position)  # tf from base_link to tcp_goal = bTg
            backing_mx = trfm.inverse_matrix(
                self.toolData.matrix)  # tf from tcp_goal to wrist = gTw
            goal_matrix = np.dot(goal_matrix, backing_mx)  # bTg * gTw = bTw
            goal_pose = Conntext.matrix_to_pose(goal_matrix, b_link)

        # self.interface.send_info("Publishing goal for tool0 in base_link: {}".format(goal_pose.pose), 1)
        if self.motion_permitted:
            self.interface.publish_command_position(goal_pose)

    @staticmethod
    def to_homogeneous(quat, point):
        """Takes a quaternion and msg.Point and outputs a homog. tf matrix.
        :param quat: (geometry_msgs.Quaternion) Orientation information.
        :param point: (geometry.msgs.Point) Position information.
        :return: (np.Array()) 4x4 Homogeneous transform matrix.
        """
        # TODO candidate for Utils
        output = trfm.quaternion_matrix(np.array([quat.x, quat.y, quat.z, quat.w]))
        output[0][3] = point.x
        output[1][3] = point.y
        output[2][3] = point.z
        return output

    @staticmethod
    def matrix_to_pose(input, base_frame):
        """Converts matrix into a pose.
        :param input: (np.Array) 4x4 homogeneous transformation matrix
        :param base_frame: (string) base frame for new pose.
        :return: (geometry_msgs.PoseStamped) Pose based on input.
        """
        output = PoseStamped()
        # output.header.stamp = self.interface.get_unified_time()
        output.header.frame_id = base_frame

        quat = trfm.quaternion_from_matrix(input)
        output.pose.orientation.x = quat[0]
        output.pose.orientation.y = quat[1]
        output.pose.orientation.z = quat[2]
        output.pose.orientation.w = quat[3]
        output.pose.position.x = input[0][3]
        output.pose.position.y = input[1][3]
        output.pose.position.z = input[2][3]
        return output

    @staticmethod
    def create_adjoint_representation(T_ab=None, R_ab=None, P_ab=None):
        """Convert homogeneous transform (T_ab) or a combination rotation matrix (R_ab) and pose (P_ab) 
        into the adjoint representation. This can be used to transform wrenches (e.g., force and torque) between frames.
        If T_ab is provided, R_ab and P_ab will be ignored.
        :param T_ab: (np.Array) 4x4 homogeneous transformation matrix representing frame 'b' relative to frame 'a'
        :param R_ab: (np.Array) 3x3 rotation matrix representing frame 'b' relative to frame 'a'
        :param P_ab: (np.Array) 3x1 pose representing frame 'b' relative to frame 'a'
        :return Ad_T: (np.Array) 6x6 adjoint representation of the transformation
        """
        # Accomodation for input R_ab and P_ab
        if (type(T_ab) == type(None)):
            T_ab = RpToTrans(R_ab, P_ab)

        Ad_T = homogeneous_to_adjoint(T_ab)
        return Ad_T

    @staticmethod
    def wrenchToArray(wrench: Wrench):
        """Restructures wrench object into numpy array with order needed by wrench reinterpretation math, namely, torque first then forces.
        :param wrench: (geometry_msgs.Wrench) Input wrench.
        :return: (np.Array) 1x6 numpy array 
        """
        return np.array(
            [wrench.torque.x, wrench.torque.y, wrench.torque.z, wrench.force.x, wrench.force.y, wrench.force.z])

    @staticmethod
    def arrayToWrench(array: np.ndarray) -> np.ndarray:
        """Restructures output 1x6 mathematical array representation of a wrench into a wrench object.
        :param wrench: (np.Array) 1x6 numpy array 
        :return: (geometry_msgs.Wrench) Return wrench.
        """
        return Wrench(Point(*list(array[3:])), Point(*list(array[:3])))

    @staticmethod
    def transform_wrench(transform: TransformStamped, wrench: Wrench, invert: bool = False,
                         log: bool = False) -> np.ndarray:
        """Transform a wrench object by the given transform object.
        :param transform: (geometry_msgs.TransformStamped) Transform to apply
        :param wrench: (geometry_msgs.Wrench) Wrench object to transform.
        :param invert: (bool) Whether to interpret the tansformation's inverse, i.e. transform "from child to parent" instead of "from parent to child"
        :return: (geometry.msgs.Wrench) changed wrench
        """
        matrix = Conntext.to_homogeneous(transform.transform.rotation, transform.transform.translation)
        if log:
            print(Fore.RED + " Transform passed in is " + str(
                transform) + " and matrix passed in is \n" + str(matrix) + Style.RESET_ALL, 2)
        if invert:
            matrix = trfm.inverse_matrix(matrix)
        return Conntext.transform_wrench_by_matrix(matrix, Conntext.wrenchToArray(wrench))

    @staticmethod
    def transform_wrench_by_matrix(T_ab: np.ndarray, wrench: np.ndarray) -> np.ndarray:
        """Use the homogeneous transform (T_ab) to transform a given wrench using an adjoint transformation (see create_adjoint_representation).
        :param T_ab: (np.Array) 4x4 homogeneous transformation matrix representing frame 'b' relative to frame 'a'
        :param wrench: (np.Array) 6x1 representation of a wrench relative to frame 'a'. This should include forces and torques as np.array([torque, force])
        :return wrench_transformed: (geometry_msgs.msg.Wrench) 6x1 representation of a wrench relative to frame 'b'. This should include forces and torques as np.array([torque, force])
        """

        Ad_T = Conntext.create_adjoint_representation(T_ab)
        wrench_transformed = np.matmul(Ad_T.T, wrench)
        return Conntext.arrayToWrench(wrench_transformed)

    @staticmethod
    def matrix_to_tf(input: np.ndarray, base_frame: str, child_frame: str):
        """Converts matrix back into a TF.
        :param input: (np.Array) 4x4 homogeneous transformation matrix
        :param base_frame: (string) base frame for new pose.
        :return: (geometry_msgs.TransformStamped) Transform based on input.
        """
        pose = Conntext.matrix_to_pose(input, base_frame)
        output = Conntext.swap_pose_tf(pose, child_frame)
        return output

    @staticmethod
    def swap_pose_tf(input: PoseStamped, child_frame: str) -> TransformStamped:
        """Swaps pose for tf and vice-versa.
        :param input: (geometry_msgs.PoseStamped or geometry_msgs.TransformStamped) Input data type.
        :param child_frame: (string) Child frame name if converting Pose to Transform.
        :return: (geometry_msgs.TransformStamped or geometry_msgs.PoseStamped) Output data, of the other type from input.
        """
        if ('PoseStamped' in str(type(input))):
            output = TransformStamped()
            output.header = input.header
            # output.transform = input.pose
            [output.transform.translation.x, output.transform.translation.y, output.transform.translation.z] = [
                input.pose.position.x, input.pose.position.y, input.pose.position.z]
            output.transform.rotation = input.pose.orientation

            output.child_frame_id = child_frame
            return output
        else:
            if ('TransformStamped' in str(type(input))):
                output = PoseStamped()
                output.header = input.header
                output.pose = input.transform
                return output
        print("Invalid input to swap_pose_tf !!!")

    def create_wrench(self, force: list, torque: list) -> WrenchStamped:
        """Composes a standard wrench object from human-readable vectors.
        :param force: (list of floats) x,y,z force values
        :param torque: (list of floats) torques about x,y,z
        :return: (geometry.msgs.WrenchStamped) Output wrench.
        """
        wrench_stamped = WrenchStamped()
        wrench_stamped.wrench = Wrench(Point(*force), Point(*torque))
        # create header
        wrench_stamped.header.stamp = self.interface.get_unified_time()
        wrench_stamped.header.frame_id = "base_link"
        #
        # # create wrench
        # wrench.force.x, wrench.force.y, wrench.force.z = force
        # wrench.torque.x, wrench.torque.y, wrench.torque.z = torque
        #
        # # self._seq+=1
        # wrench_stamped.wrench = wrench

        return wrench_stamped

    def update_average_wrench(self) -> None:
        """Create a very simple moving average of the incoming wrench readings and store it as self.average.wrench.
        """
        self.current_wrench = self.interface.get_current_wrench()
        self._average_wrench_gripper = self.filters.average_wrench(self.current_wrench.wrench)

        # Get current angle from gripper to hole:
        transform_world_rotation: TransformStamped = self.interface.get_transform('tool0', 'target_hole_position')

        # We want to rotate this only, not reinterpret F/T components.
        # We reinterpret based on the position of the TCP (but ignore the relative rotation). In addition, the wrench
        # is internally measured at the load cell and has a built-in transformation to tool0 which is 5cm forward.
        # We have to undo that transformation to get accurate transformation.
        relative_translation = self.toolData.transform.transform.translation
        offset = Point(relative_translation.x,
                       relative_translation.y,
                       relative_translation.z - .05)

        transform_world_rotation.transform.translation = offset

        # Execute reinterpret-to-tcp and rotate-to-world simultaneously:
        self._average_wrench_world = Conntext.transform_wrench(transform_world_rotation,
                                                                    self._average_wrench_gripper)  # This works

        # Output the wrench for debug visualization
        guy = self.create_wrench([0, 0, 0], [0, 0, 0])
        guy.wrench = self._average_wrench_world
        guy.header.frame_id = "target_hole_position"
        self.interface.publish_averaged_wrench(guy)

    def update_avg_speed(self) -> None:
        """Updates a simple moving average of robot tcp speed in m/s. A speed is calculated from the difference between a
         previous pose (.1 s in the past) and the current pose; this speed is filtered and stored as self.average_speed.
        """

        distance, time = self.interface.get_transform_change_over_time(
            self.toolData.frame_name,
            "base_link",
            .1)
        if (time is not None and time > 0.0):  # Update only if we're using a new pose; also, avoid divide by zero
            speedDiff = distance / time
            self.average_speed = self.filters.average_speed(speedDiff)
            # self.interface.send_info("Speed is {}".format(self.average_speed), 2)
            if (np.linalg.norm(self.average_speed) > self.params['robot']['hard_speed_limit']):
                self.interface.send_error("Speed too high, quitting. Speed: {} \nTotal:{}".format(
                    self.average_speed, np.linalg.norm(self.average_speed)))
                quit()

    def as_array(self, vec: Point) -> np.ndarray:
        """Takes a Point and returns a Numpy array.
        :param vec: (geometry_msgs.Point) Vector in serialized ROS format.
        :return: (numpy.Array) Vector in 3x1 numpy array format.
        """
        # insist that we get a 1D array returned
        return np.array([vec.x, vec.y, vec.z]).reshape(-1, )

    def vectorRegionCompare_symmetrical(self, input: np.ndarray, bounds_max: list) -> bool:
        """See ``vectorRegionCompare``_. Compares an input to boundaries element-wise. Essentially checks whether a vector
         is within a rectangular region. This version assumes min values to be the negative of max values.
        :param input: (list of floats) x,y,z of a vector to check.
        :param bounds_max: (list of floats) x,y,z max value of each element.
        :return: (bool) Whether the vector falls within the region.
        """

        # initialize a minimum list
        bounds_min = [0, 0, 0]
        # Each min value is the negative of the max value
        # Create bounds_min to be the negative of bounds_max. symmetrical, duh....
        bounds_min[0] = bounds_max[0] * -1.0
        bounds_min[1] = bounds_max[1] * -1.0
        bounds_min[2] = bounds_max[2] * -1.0
        return self.vectorRegionCompare(input, bounds_max, bounds_min)

    def vectorRegionCompare(self, input: list, bounds_max: list, bounds_min: list) -> bool:
        """.. vectorRegionCompare Compares an input to boundaries element-wise. Essentially checks whether a vector is within a rectangular region.
        # bounds_max and bounds_min let you set a range for each dimension.
        :param input: (list of floats) x,y,z of a vector to check.
        :param bounds_max: (list of floats) x,y,z max value of each element.
        :param bounds_min: (list of floats) x,y,z min value of each element.
        :return: (bool) Whether the vector falls within the region. 
        """
        # Simply compares abs. val.s of input's elements to a vector of maximums and returns whether it exceeds
        # if(symmetrical):
        #    bounds_min[0], bounds_min[1], bounds_min[2] = bounds_max[0] * -1, bounds_max[1] * -1, bounds_max[2] * -1
        # TODO - convert to a process of numpy arrays! They process way faster because that library is written in C++
        # Note - actually Numpy's allclose() method may be perfect here.

        if (bounds_max[0] >= input[0] >= bounds_min[0]):
            if (bounds_max[1] >= input[1] >= bounds_min[1]):
                if (bounds_max[2] >= input[2] >= bounds_min[2]):
                    return True
        return False

    def checkIfStatic(self) -> bool:
        maxSpeeds = np.array(self.params['robot']["speed_static"])
        res = np.allclose(abs(self.average_speed), np.zeros(3), atol=abs(maxSpeeds))

        return res
        # return self.vectorRegionCompare_symmetrical(self.average_speed, maxSpeeds)

    def checkIfColliding(self, commandedForce: np.ndarray, deadzoneRadius: list = [4, 4, 3],
                         relativeScaling: float = .1) -> bool:
        """Checks if  an equal and opposite reaction force is stopping acceleration in all directions - this would indicate there is a static obstacle in collision with the tcp.
        """
        force = self.as_array(self._average_wrench_world.force).reshape(3)
        res = np.allclose(force, -1 * commandedForce, atol=deadzoneRadius, rtol=relativeScaling)
        return res

    # TODO: Make the parameters of function part of the constructor or something...
    def force_cap_check(self):
        """Checks whether any forces or torques are dangerously high. There are two levels of response:
            *Elevated levels of force cause this program to pause for 1s. If forces remain high after pause, 
            the system will enter a freewheeling state
            *Dangerously high forces will kill this program immediately to prevent damage.
        :return: (Bool) True if all is safe; False if a warning stop is requested.
        """
        force_limits = self.params['robot']['force_setpoints']
        # Calculate acceptable torque from transverse forces by taking the moment arm to tcp:
        radius = np.linalg.norm(self.as_array(self.toolData.transform.transform.translation))
        # Set a minimum radius to always permit some torque
        radius = max(3, radius)
        warning_torque = [force_limits['transverse']['warning'][a] * radius for a in range(3)]
        danger_torque = [force_limits['transverse']['dangerous'][b] * radius for b in range(3)]
        force_array  = self.as_array(self.current_wrench.wrench.force)
        torque_array = self.as_array(self.current_wrench.wrench.torque)

        if (not (self.vectorRegionCompare_symmetrical(force_array, force_limits['direct']['dangerous'])
                 and self.vectorRegionCompare_symmetrical(torque_array, danger_torque))):
            self.interface.send_error("*Very* high force/torque detected! {}".format(self.current_wrench.wrench), 2)
            quit()  # kills the program. Since the node is required, it kills the ROS application.
            return True
        if (self.vectorRegionCompare_symmetrical(force_array, force_limits['direct']['warning'])):
            if (self.vectorRegionCompare_symmetrical(torque_array, warning_torque)):
                return True

        self.interface.send_error("High force/torque detected! \nForce: {}\nTorque: {}".format(force_array, torque_array))
        # Forces and torques within acceptable levels. Reset highForceWarning
        if (self.highForceWarning):
            self.highForceWarning = False
            return False
        else:
            self.interface.send_error("Sleeping for 1s to damp oscillations...")
            self.highForceWarning = True
            time.sleep(1)
            # Makes the system to stop for a second in hopes that it prevents higher forces/torques.
            # May not be helping.
        return True

    @current_pose.setter
    def current_pose(self, value):
        self._current_pose = value

    @current_wrench.setter
    def current_wrench(self, value):
        self._current_wrench = value
