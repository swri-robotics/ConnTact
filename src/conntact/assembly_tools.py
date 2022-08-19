# Copyright 2021 Southwest Research Institute
# Licensed under the Apache License, Version 2.0

# Imports for ros
import time

import rospy
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
from conntact.assembly_utils import AssemblyFilters

class ToolData():
    def __init__(self):
        self.name = ""
        self.frame_name = ""
        self.transform = None
        self.matrix = None
        self.validToolsDict = None

class AssemblyTools():

    def __init__(self, interface: ConntactInterface, conntact_params="conntact_params"):
        # Save a reference to the interface.
        self.interface = interface
        self.params = self.interface.load_yaml_file(conntact_params)

        # save off the important stuff that's accessed every loop cycle
        force_dangerous = [55, 55, 65]  # Force value which kills the program. Rel. to gripper.
        force_transverse_dangerous = np.array([30, 30,
                                               30])  # Force value transverse to the line from the TCP to the force sensor which kills the program. Rel. to gripper.
        force_warning = [40, 40, 50]  # Force value which pauses the program. Rel. to gripper.
        force_transverse_warning = np.array([20, 20,
                                             20])  # torque value transverse to the line from the TCP to the force sensor which kills the program. Rel. to gripper.
        self.max_force_error = [4, 4, 4]  # Allowable error force with no actual loads on the gripper.
        self.cap_check_forces = force_dangerous, force_transverse_dangerous, force_warning, force_transverse_warning
        # self.cap_check_forces = self.params["robot"]["force_setpoints"]
        # Instantiate the dictionary of frames which are always required for tasks.
        self.reference_frames = {"tcp": TransformStamped(), "target_hole_position": TransformStamped()}
        self._start_time = self.interface.get_unified_time()


        # Read in yaml config file
        # path = self.interface.get_package_path()
        #
        # def load_yaml_file(filename):
        #     with open(path + '/config/' + filename + '.yaml') as stream:
        #         try:
        #             info = yaml.safe_load(stream)
        #             # print(info)
        #             self.params.update(info)
        #         except yaml.YAMLError as exc:
        #             print(exc)
        #
        # load_yaml_file(conntact_params)
        # load_yaml_file("peg_in_hole_params")
        # print("Full config dict: {}".format(self.params))

        self._rate_selected = self.params["framework"]["rate"]
        self.highForceWarning = False
        # self._seq                   = 0

        # Initialize filtering class
        self.filters = AssemblyFilters(5, self._rate_selected)

        self.tool_data = dict()
        self.toolData = ToolData()
        """ Dictionary of transform/ matrix transformation dictionary which contains each TCP configuration loaded from the YAML. It is automatically populated in readYAML(). Access info by invoking: 

        self.tool_data[*tool name*]["transform"] = (geometry_msgs.TransformStamped) Transform from tool0 (robot wrist flange) to tcp location.
        self.tool_data[*tool name*]["matrix"] = (np.array()) 4x4 homogeneous transformation matrix of same transform.
        """

        # print(Fore.RED + Back.BLUE + "AssemblyTools sleeping, estop now if you don't want motion..." + Style.RESET_ALL)
        # time.sleep(5)
        self.readYAML()

        # loop parameters
        # self.wrench_vec = self.get_command_wrench([0, 0, 0])
        self.next_trigger = ''  # Empty to start. Each callback should decide what next trigger to implement in the main loop
        self.switch_state = False

        # initialize loop parameters

        self.current_pose = self.get_current_pos()
        self.current_wrench = self.create_wrench([0, 0, 0], [0, 0, 0])
        self._average_wrench_gripper = self.create_wrench([0, 0, 0], [0, 0, 0]).wrench
        self._average_wrench_world = Wrench()
        self.average_speed = np.array([0.0,0.0,0.0])

    def readYAML(self):
        """Read data from job config YAML and make certain calculations for later use. Stores peg frames in dictionary tool_data
        """

        # job parameters moved in from the peg_in_hole_params.yaml file
        # 'peg_4mm' 'peg_8mm' 'peg_10mm' 'peg_16mm'
        # 'hole_4mm' 'hole_8mm' 'hole_10mm' 'hole_16mm'
        self.target_peg = self.config['task']['target_peg']
        self.target_hole = self.config['task']['target_hole']
        self.activeTCP = self.config['task']['starting_tcp']

        self.read_board_positions()

        self.read_peg_hole_dimensions()

        # Spiral parameters
        self._spiral_params = self.config['algorithm']['spiral_params']

        # Calculate transform from TCP to peg corner
        peg_locations = self.config['objects'][self.target_peg]['grasping_locations']

        # Set up tool_data.
        self.toolData.name = self.activeTCP
        self.toolData.validToolsDict = peg_locations

        self.select_tool(self.activeTCP)
        print("Select Tool successful!")

        self.surface_height = self.config['task']['assumed_starting_height']  # Starting height assumption
        self.restart_height = self.config['task']['restart_height']  # Height to restart

    def read_board_positions(self):
        """ Calculates pose of target hole relative to robot base frame.
        """
        temp_z_position_offset = 207  # Our robot is reading Z positions wrong on the pendant for some reason.
        task_pos = list(np.array(self.config['environment_state']['task_frame']['position']))
        task_pos[2] = task_pos[2] + temp_z_position_offset
        task_ori = self.config['environment_state']['task_frame']['orientation']
        hole_pos = list(np.array(self.config['objects'][self.target_hole]['local_position']))
        hole_pos[2] = hole_pos[2] + temp_z_position_offset
        hole_ori = self.config['objects'][self.target_hole]['local_orientation']

        # Set up target hole pose
        tf_robot_to_task_board = AssemblyTools.get_tf_from_yaml(task_pos, task_ori, "base_link", "task_board")
        pose_task_board_to_hole = AssemblyTools.get_pose_from_yaml(hole_pos, hole_ori, "base_link")
        self.target_hole_pose = tf2_geometry_msgs.do_transform_pose(pose_task_board_to_hole, tf_robot_to_task_board)
        # self.target_broadcaster = tf2_geometry_msgs.do_transform_pose(self.pose_task_board_to_hole, self.tf_robot_to_task_board)
        targetHoleTF = AssemblyTools.swap_pose_tf(self.target_hole_pose, "target_hole_position")
        self.reference_frames['target_hole_position'] = targetHoleTF
        # self.send_reference_TFs()
        self.x_pos_offset = self.target_hole_pose.pose.position.x
        self.y_pos_offset = self.target_hole_pose.pose.position.y

    def read_peg_hole_dimensions(self):
        """Read peg and hole data from YAML configuration file.
        """
        peg_dims = self.config['objects'][self.target_peg]['dimensions']
        hole_dims = self.config['objects'][self.target_hole]['dimensions']
        self.hole_depth = peg_dims['min_insertion_depth'] / 1000
        # setup, run to calculate useful values based on params:
        clearance_max = hole_dims['upper_tolerance'] - peg_dims['lower_tolerance']  # calculate the total error zone;
        clearance_min = hole_dims['lower_tolerance'] + peg_dims['upper_tolerance']  # calculate minimum clearance;     =0
        clearance_avg = .5 * (clearance_max - clearance_min)  # provisional calculation of "wiggle room"
        self.safe_clearance = (hole_dims['diameter'] - peg_dims['diameter'] + clearance_min) / 2000;  # = .2 *radial* clearance i.e. on each side.

    def send_reference_TFs(self):
        self.interface.register_frames(list(self.reference_frames.values()))


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

        output_pose = AssemblyTools.get_pose_from_yaml(pos, ori, base_frame)  # tf_task_board_to_hole
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
        output_pose.header.stamp = self.interface.get_unified_time()
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
            gripper_to_tool_tip_transform = AssemblyTools.get_tf_from_yaml(
                self.toolData.validToolsDict[tool_name]['pose'],
                self.toolData.validToolsDict[tool_name]['orientation'],
                "tool0_to_gripper_tip_link", self.toolData.frame_name)
            self.reference_frames['tcp'] = gripper_to_tool_tip_transform
            self.send_reference_TFs()
            # get the transform from tool0 (which CartesianControllers treats as the root frame) to the TCP
            self.toolData.transform = self.interface.get_transform(self.toolData.frame_name, "tool0")
            self.toolData.matrix    = AssemblyTools.to_homogeneous(
                self.toolData.transform.transform.rotation,
                self.toolData.transform.transform.translation
                )
            print("New tool position selected: {}".format(self.toolData.name))

        else:
            self.interface.send_error("Tool selection key error! No key '" +
                                      tool_name + "' in tool dictionary.", 2)

    def spiral_search_motion(self, frequency=.15, min_amplitude=.002, max_cycles=62.83185):
        """Generates position, orientation offset vectors which describe a plane spiral about z; 
        Adds this offset to the current approach vector to create a searching pattern. Constants come from Init;
        x,y vector currently comes from x_ and y_pos_offset variables.
        """
        curr_time = self.interface.get_unified_time() - self._start_time
        curr_time_numpy = np.double(curr_time.to_sec())
        curr_amp = min_amplitude + self.safe_clearance * np.mod(2.0 * np.pi * frequency * curr_time_numpy, max_cycles);
        x_pos = curr_amp * np.cos(2.0 * np.pi * frequency * curr_time_numpy)
        y_pos = curr_amp * np.sin(2.0 * np.pi * frequency * curr_time_numpy)
        x_pos = x_pos + self.x_pos_offset
        y_pos = y_pos + self.y_pos_offset
        z_pos = self.current_pose.transform.translation.z
        pose_position = [x_pos, y_pos, z_pos]
        pose_orientation = [0, 1, 0, 0]  # w, x, y, z

        return [pose_position, pose_orientation]

    def linear_search_position(self, direction_vector=[0, 0, 0], desired_orientation=[0, 1, 0, 0]):
        """Generates a command pose vector which causes the robot to hold a certain orientation
         and comply in z while maintaining the approach vector along x_ and y_pos_offset.
        :param direction_vector: (list of floats) vector directional offset from normal position. Causes constant motion in z.
        :param desired_orientation: (list of floats) quaternion parameters for orientation. 
        """
        # pose_position = self.current_pose.transform.translation
        pose_position = Vector3()
        pose_position.x, pose_position.y, pose_position.z = \
            self.as_array(self.current_pose.transform.translation)

        pose_position.x = self.x_pos_offset + direction_vector[0]
        pose_position.y = self.y_pos_offset + direction_vector[1]
        pose_position.z = pose_position.z + direction_vector[2]
        pose_orientation = desired_orientation

        # self.interface.send_info(
        #     Fore.CYAN + "\nlinear_search_position requested by: {}\nReturning:\n{}".format(
        #         inspect.stack()[1][3],
        #         pose_position
        #     ) + Style.RESET_ALL)

        return [[pose_position.x, pose_position.y, pose_position.z], pose_orientation]

    def full_compliance_position(self, direction_vector=[0, 0, 0], desired_orientation=[0, 1, 0, 0]):
        """Generates a command pose vector which causes the robot to hold a certain orientation
         and comply translationally in all directions.
        :param direction_vector: (list of floats) vector directional offset from normal position. Causes constant motion.
        :param desired_orientation: (list of floats) quaternion parameters for orientation. 
        """
        # pose_position = self.current_pose.transform.translation
        pose_position = Vector3()
        pose_position.x, pose_position.y, pose_position.z = \
            self.as_array(self.current_pose.transform.translation)
        pose_position.x = pose_position.x + direction_vector[0]
        pose_position.y = pose_position.y + direction_vector[1]
        pose_position.z = pose_position.z + direction_vector[2]
        pose_orientation = desired_orientation
        return [[pose_position.x, pose_position.y, pose_position.z], pose_orientation]

    def get_current_pos(self):
        """Read in current pose from robot base to activeTCP.        
        """
        transform = TransformStamped()
        transform = self.interface.get_transform(self.toolData.frame_name,
                                                 "base_link")
        # To help debug:
        # self.interface.send_info(
        #     Fore.BLUE + "\nCurrent pos: \n{}\nRequested by:\n{}\n".format(
        #         transform.transform.translation,
        #         inspect.stack()[1][3]
        #     ) + Style.RESET_ALL)
        return transform

    def arbitrary_axis_comply(self, direction_vector = [0,0,1], desired_orientation = [0, 1, 0, 0]):
        """Generates a command pose vector which causes the robot to hold a certain orientation
         and comply in one dimension while staying on track in the others.
        :param desiredTaskSpacePosition: (array-like) vector indicating hole position in robot frame
        :param direction_vector: (array-like list of bools) vector of bools or 0/1 values to indicate which axes comply and which try to stay the same as those of the target hole position.
        :param desired_orientation: (list of floats) quaternion parameters for orientation; currently disabled because changes in orientation are dangerous and unpredictable. Use TCPs instead.
        """

        #initially set the new command position to be the current physical (disturbed) position
        #This allows movement allowed by outside/command forces to move the robot at a steady rate.

        pose_position = Vector3()
        pose_position.x, pose_position.y, pose_position.z =\
            self.as_array(self.current_pose.transform.translation)
        if(not direction_vector[0]):
            pose_position.x = self.target_hole_pose.pose.position.x
        if(not direction_vector[1]):
            pose_position.y = self.target_hole_pose.pose.position.y
        if(not direction_vector[2]):
            pose_position.z = self.target_hole_pose.pose.position.z
        pose_orientation = [0, 1, 0, 0]
        # self.interface.send_info(
        #     Fore.CYAN + "\nArbitrary_axis_comply requested by: {}\nReturning:\n{}".format(
        #         inspect.stack()[1][3],
        #         pose_position
        #     ) + Style.RESET_ALL)
        return [[pose_position.x, pose_position.y, pose_position.z], pose_orientation]

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
        result_wrench = self.create_wrench(input_vec[:3], input_vec[3:])

        transform_world_to_gripper: TransformStamped = self.interface.get_transform('target_hole_position', 'tool0')
        tcp_position = self.toolData.transform.transform.translation
        offset = Point(-1 * tcp_position.x, -1 * tcp_position.y, -1 * tcp_position.z - .05)

        transform_world_to_gripper.transform.translation = offset

        # Execute reinterpret-to-tcp and rotate-to-world simultaneously:
        result_wrench.wrench = AssemblyTools.transform_wrench(transform_world_to_gripper,
                                                              result_wrench.wrench)  # This works

        self.interface.publish_command_wrench(result_wrench)

    @staticmethod
    def list_from_quat(quat):
        return [quat.x, quat.y, quat.z, quat.w]

    @staticmethod
    def list_from_point(point):
        return [point.x, point.y, point.z]

    def publish_pose(self, pose_stamped_vec):
        """Takes in vector representations of position 
        :param pose_stamped_vec: (list of floats) List of parameters for pose with x,y,z position and orientation quaternion
        """
        # Ensure controller is loaded
        # self.check_controller(self.controller_name)

        if (pose_stamped_vec is None):
            self.interface.send_info("Command position not initialized yet...")
            return


        # Create poseStamped msg
        goal_pose = PoseStamped()

        # Set the position and orientation
        point = Point()
        quaternion = Quaternion()

        # point.x, point.y, point.z = position
        point.x, point.y, point.z = pose_stamped_vec[0][:]
        goal_pose.pose.position = point

        quaternion.w, quaternion.x, quaternion.y, quaternion.z = pose_stamped_vec[1][:]
        goal_pose.pose.orientation = quaternion

        # Set header values
        goal_pose.header.stamp = self.interface.get_unified_time()
        goal_pose.header.frame_id = "base_link"

        if (self.activeTCP != "tool0"):
            # Convert pose in TCP coordinates to assign wrist "tool0" position for controller

            b_link = goal_pose.header.frame_id
            goal_matrix = AssemblyTools.to_homogeneous(goal_pose.pose.orientation,
                                                       goal_pose.pose.position)  # tf from base_link to tcp_goal = bTg
            backing_mx = trfm.inverse_matrix(
                self.toolData.matrix)  # tf from tcp_goal to wrist = gTw
            goal_matrix = np.dot(goal_matrix, backing_mx)  # bTg * gTw = bTw
            goal_pose = AssemblyTools.matrix_to_pose(goal_matrix, b_link)

        # self.interface.send_info(
        #     Fore.BLUE + "Pose_Stamped_Vec:\n{}\nPublishing goal pos:\n{}\n".format(
        #         pose_stamped_vec,
        #         goal_pose.pose.position
        #     ) + Style.RESET_ALL)

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
        output.header.stamp = self.interface.get_unified_time()
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
        matrix = AssemblyTools.to_homogeneous(transform.transform.rotation, transform.transform.translation)
        if log:
            print(Fore.RED + " Transform passed in is " + str(
                transform) + " and matrix passed in is \n" + str(matrix) + Style.RESET_ALL, 2)
        if invert:
            matrix = trfm.inverse_matrix(matrix)
        return AssemblyTools.transform_wrench_by_matrix(matrix, AssemblyTools.wrenchToArray(wrench))

    @staticmethod
    def transform_wrench_by_matrix(T_ab: np.ndarray, wrench: np.ndarray) -> np.ndarray:
        """Use the homogeneous transform (T_ab) to transform a given wrench using an adjoint transformation (see create_adjoint_representation).
        :param T_ab: (np.Array) 4x4 homogeneous transformation matrix representing frame 'b' relative to frame 'a'
        :param wrench: (np.Array) 6x1 representation of a wrench relative to frame 'a'. This should include forces and torques as np.array([torque, force])
        :return wrench_transformed: (geometry_msgs.msg.Wrench) 6x1 representation of a wrench relative to frame 'b'. This should include forces and torques as np.array([torque, force])
        """

        Ad_T = AssemblyTools.create_adjoint_representation(T_ab)
        wrench_transformed = np.matmul(Ad_T.T, wrench)
        return AssemblyTools.arrayToWrench(wrench_transformed)

    @staticmethod
    def matrix_to_tf(input: np.ndarray, base_frame: str, child_frame: str):
        """Converts matrix back into a TF.
        :param input: (np.Array) 4x4 homogeneous transformation matrix
        :param base_frame: (string) base frame for new pose.
        :return: (geometry_msgs.TransformStamped) Transform based on input.
        """
        pose = AssemblyTools.matrix_to_pose(input, base_frame)
        output = AssemblyTools.swap_pose_tf(pose, child_frame)
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
        wrench = Wrench()

        # create wrench
        wrench.force.x, wrench.force.y, wrench.force.z = force
        wrench.torque.x, wrench.torque.y, wrench.torque.z = torque
        # create header
        # wrench_stamped.header.seq = self._seq
        wrench_stamped.header.stamp = self.interface.get_unified_time()
        wrench_stamped.header.frame_id = "base_link"
        # self._seq+=1
        wrench_stamped.wrench = wrench

        return wrench_stamped

    def update_average_wrench(self) -> None:
        """Create a very simple moving average of the incoming wrench readings and store it as self.average.wrench.
        """
        self.current_wrench = self.interface.get_current_wrench()
        self._average_wrench_gripper = self.filters.average_wrench(self.current_wrench.wrench)

        # Get current angle from gripper to hole:
        transform_world_rotation: TransformStamped = self.interface.get_transform('tool0', 'target_hole_position')

        # We want to rotate this only, not reinterpret F/T components.
        # We reinterpret based on the position of the TCP (but ignore the relative rotation). In addition, the wrench is internally measured at the load cell and has a built-in transformation to tool0 which is 5cm forward. We have to undo that transformation to get accurate transformation.

        relative_translation = self.toolData.transform.transform.translation
        offset = Point(relative_translation.x,
                       relative_translation.y,
                       relative_translation.z - .05)

        transform_world_rotation.transform.translation = offset

        # Execute reinterpret-to-tcp and rotate-to-world simultaneously:
        self._average_wrench_world = AssemblyTools.transform_wrench(transform_world_rotation,
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
            self.interface.send_info("Speed is {}".format(self.average_speed), 2)
            if (np.linalg.norm(self.average_speed) > .035):
                self.interface.send_error("Speed too high, quitting.")
                quit()

    def publish_plotted_values(self, stateInfo) -> None:
        """Publishes critical data for plotting node to process.
        """

        items = dict()
        # Send a dictionary as plain text to expose some additional info
        items["status_dict"] = dict(
            {stateInfo, ('tcp_name', str(self.toolData.frame_name))})
        if (self.surface_height != 0.0):
            # If we have located the work surface
            items["status_dict"]['surface_height'] = str(self.surface_height)

        # self.status_pub.publish(str(status_dict))

        # self.avg_wrench_pub.publish(self._average_wrench_world)
        items["_average_wrench_world"] = self._average_wrench_world

        # self.avg_speed_pub.publish(Point(self.average_speed[0], self.average_speed[1], self.average_speed[2]))
        items["average_speed"] = self.average_speed

        items["current_pose"] = self.current_pose.transform.translation

        self.interface.publish_plotting_values(items)

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

        # rospy.logerr("Bounds and stuff: " +str(input) + str(bounds_max) + str(bounds_min))
        if (bounds_max[0] >= input[0] >= bounds_min[0]):
            if (bounds_max[1] >= input[1] >= bounds_min[1]):
                if (bounds_max[2] >= input[2] >= bounds_min[2]):
                    return True
        return False

    def checkIfStatic(self, maxSpeeds: np.ndarray) -> bool:
        res = np.allclose(abs(self.average_speed), np.zeros(3), atol=abs(maxSpeeds))

        return res
        # return self.vectorRegionCompare_symmetrical(self.average_speed, maxSpeeds)

    def checkIfColliding(self, commandedForce: np.ndarray, deadzoneRadius: list = [4, 4, 3],
                         relativeScaling: float = .1) -> bool:
        """Checks if  an equal and opposite reaction force is stopping acceleration in all directions - this would indicate there is a static obstacle in collision with the tcp.
        """

        force = self.as_array(self._average_wrench_world.force).reshape(3)
        # self.interface.send_info("Commanded force: {}\n Perceived force:{}".format(
        #     commandedForce, force)
        # )
        res = np.allclose(force, -1 * commandedForce, atol=deadzoneRadius, rtol=relativeScaling)

        # rospy.loginfo_throttle(1,Fore.BLUE +  "Collision checking force " + str(force) + " against command " + str(commandedForce*-1) + ' with a result of ' + str(res) + " and the difference is " + str(force + commandedForce) + Style.RESET_ALL)

        return res
        # if(type(lowerThresholds) == type(None)):
        #     lowerThresholds = -1 * upperThresholds
        # return self.vectorRegionCompare(self.as_array(self._average_wrench_world.force)-commandedForce, upperThresholds, lowerThresholds)

    # TODO: Make the parameters of function part of the constructor or something...
    def force_cap_check(self):
        """Checks whether any forces or torques are dangerously high. There are two levels of response:
            *Elevated levels of force cause this program to pause for 1s. If forces remain high after pause, 
            the system will enter a freewheeling state
            *Dangerously high forces will kill this program immediately to prevent damage.
        :return: (Bool) True if all is safe; False if a warning stop is requested.
        """
        force_limits = self.config['robot']['force_setpoints']
        # Calculate acceptable torque from transverse forces by taking the moment arm to tcp:
        radius = np.linalg.norm(self.as_array(self.toolData.transform.transform.translation))
        # Set a minimum radius to always permit some torque
        radius = max(3, radius)
        rospy.loginfo_once("For TCP " + self.activeTCP + " moment arm is coming out to " + str(radius))
        warning_torque = [force_limits['transverse']['warning'][a] * radius for a in range(3)]
        danger_torque = [force_limits['transverse']['dangerous'][b] * radius for b in range(3)]
        rospy.loginfo_once("So torques are limited to  " + str(warning_torque) + str(danger_torque))
        force_array  = self.as_array(self.current_wrench.wrench.force)
        torque_array = self.as_array(self.current_wrench.wrench.torque)

        if (not (self.vectorRegionCompare_symmetrical(force_array, force_limits['direct']['dangerous'])
                 and self.vectorRegionCompare_symmetrical(torque_array, danger_torque))):
            rospy.logerr("*Very* high force/torque detected! " + str(self.current_wrench.wrench))
            rospy.logerr("Killing program.")
            quit()  # kills the program. Since the node is required, it kills the ROS application.
            return True
        if (self.vectorRegionCompare_symmetrical(force_array, force_limits['direct']['warning'])):
            if (self.vectorRegionCompare_symmetrical(torque_array, warning_torque)):
                return True
        rospy.logerr("High force/torque detected! \nForce: {}\nTorque: {}".format(force_array, torque_array))
        # Forces and torques within acceptable levels. Reset highForceWarning
        if (self.highForceWarning):
            self.highForceWarning = False
            return False
        else:
            rospy.logerr("Sleeping for 1s to damp oscillations...")
            self.highForceWarning = True
            time.sleep(1)
            # Makes the system to stop for a second in hopes that it prevents higher forces/torques.
            # May not be helping.
        return True
