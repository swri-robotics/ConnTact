# Copyright 2021 Southwest Research Institute
# Licensed under the Apache License, Version 2.0

# Imports for ros
from inspect import EndOfBlock
from operator import truediv
from typing import List
import rospy
# import tf
import numpy as np
import matplotlib.pyplot as plt
from colorama import Fore, Back, Style
from rospkg import RosPack
from geometry_msgs.msg import WrenchStamped, Wrench, TransformStamped, PoseStamped, Pose, Point, Quaternion, Vector3, Transform
from std_msgs.msg import String
from rospy.core import configure_logging

from sensor_msgs.msg import JointState
# from assembly_ros.srv import ExecuteStart, ExecuteRestart, ExecuteStop
from controller_manager_msgs.srv import SwitchController, LoadController, ListControllers

import tf2_ros
# import tf2
import tf2_geometry_msgs
import tf.transformations as trfm

from threading import Lock

from modern_robotics import Adjoint as homogeneous_to_adjoint, RpToTrans

class AssemblyTools():

    def __init__(self, ROS_rate, start_time):
        self._wrench_pub            = rospy.Publisher('/cartesian_compliance_controller/target_wrench', WrenchStamped, queue_size=10)
        self._pose_pub              = rospy.Publisher('cartesian_compliance_controller/target_frame', PoseStamped , queue_size=2)
        self._adj_wrench_pub        = rospy.Publisher('adjusted_wrench_force', WrenchStamped, queue_size=2)

        #for plotting node
        self.avg_wrench_pub         = rospy.Publisher("/assembly_tools/avg_wrench", Wrench, queue_size=5)
        self.avg_speed_pub          = rospy.Publisher("/assembly_tools/avg_speed", Point, queue_size=5)
        self.rel_position_pub       = rospy.Publisher("/assembly_tools/rel_position", Point, queue_size=5)

        self.status_pub             = rospy.Publisher("/assembly_tools/status", String, queue_size=5)

        self._ft_sensor_sub         = rospy.Subscriber("/cartesian_compliance_controller/ft_sensor_wrench/", WrenchStamped, self.callback_update_wrench, queue_size=2)
        # self._tcp_pub   = rospy.Publisher('target_hole_position', PoseStamped, queue_size=2, latch=True)

        #Needed to get current pose of the robot
        self.tf_buffer              = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener            = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster            = tf2_ros.StaticTransformBroadcaster()

        #Instantiate the dictionary of frames which are published to tf2. They have to be published in a single Broadcaster call to both be accessible.
        self.reference_frames       = {"tcp": TransformStamped(), "target_hole_position": TransformStamped()}

        self._rate_selected         = ROS_rate
        self._rate                  = rospy.Rate(self._rate_selected) #setup for sleeping in hz
        self._start_time            = start_time #for _spiral_search_basic_force_control and spiral_search_motion
        self.curr_time              = rospy.get_rostime() - self._start_time
        self.curr_time_numpy        = np.double(self.curr_time.to_sec())
        self.highForceWarning       = False
        self.collision_confidence   = 0
        # self._seq                   = 0

        # Initialize filtering class
        self.filters                = AssemblyFilters(5, self._rate_selected)

        self.tool_data              = dict()
        """ Dictionary of transform/ matrix transformation dictionary which contains each TCP configuration loaded from the YAML. It is automatically populated in readYAML(). Access info by invoking: 

        self.tool_data[*tool name*]["transform"] = (geometry_msgs.TransformStamped) Transform from tool0 (robot wrist flange) to tcp location.
        self.tool_data[*tool name*]["matrix"] = (np.array()) 4x4 homogeneous transformation matrix of same transform.
        """

        self.readYAML()

        #loop parameters
        self.wrench_vec  = self.get_command_wrench([0,0,0])
        self.next_trigger = '' #Empty to start. Each callback should decide what next trigger to implement in the main loop
        self.switch_state = False

        # initialize loop parameters
        self.current_pose = self.get_current_pos()
        self.pose_vec = self.full_compliance_position()
        self.current_wrench = self.create_wrench([0,0,0], [0,0,0])
        self._average_wrench_gripper = self.create_wrench([0,0,0], [0,0,0]).wrench 
        self._average_wrench_world = Wrench()
        self.average_speed = np.array([0.0,0.0,0.0])

        rospy.loginfo_once(Fore.CYAN + Back.RED + "Controllers list:\n" + str(ListControllers()) + Style.RESET_ALL);
 

    def readYAML(self):
        """Read data from job config YAML and make certain calculations for later use. Stores peg frames in dictionary tool_data
        """
        
        #job parameters moved in from the peg_in_hole_params.yaml file
        #'peg_4mm' 'peg_8mm' 'peg_10mm' 'peg_16mm'
        #'hole_4mm' 'hole_8mm' 'hole_10mm' 'hole_16mm'
        self.target_peg                 = rospy.get_param('/task/target_peg')
        self.target_hole                = rospy.get_param('/task/target_hole')
        self.activeTCP                  = rospy.get_param('/task/starting_tcp')

        self.read_board_positions()
        
        self.read_peg_hole_dimensions()

        #Spiral parameters
        self._spiral_params             = rospy.get_param('/algorithm/spiral_params')
        
        #Calculate transform from TCP to peg corner        
        self.peg_locations              = rospy.get_param('/objects/'+self.target_peg+'/grasping/pinch_grasping/locations')
        
        # Setup default zero-transform in case it needs to be referenced for consistency.
        self.tool_data['gripper_tip']   = dict()
        a                               = TransformStamped()
        a.header.frame_id               = "tool0"
        a.child_frame_id                = 'gripper_tip'
        a.transform.rotation.w          = 1
        self.tool_data['gripper_tip']['transform']    = a
        self.tool_data['gripper_tip']['matrix']       = AssemblyTools.to_homogeneous(a.transform.rotation, a.transform.translation)
        self.reference_frames['tcp']    = a
        

        for key in list(self.peg_locations):
            #Read in each listed tool position; measure their TF and store in dictionary.
            #Write the position of the peg's corner wrt the gripper tip as a reference-ready TF.
            pegTransform = AssemblyTools.get_tf_from_YAML(self.peg_locations[str(key)]['pose'], self.peg_locations[str(key)]['orientation'],
            "tool0_to_gripper_tip_link", "peg_"+str(key)+"_position")
            self.reference_frames['tcp'] = pegTransform
            self.send_reference_TFs()
            self._rate.sleep()
            a = self.tf_buffer.lookup_transform("tool0", "peg_"+str(key)+"_position", rospy.Time(0), rospy.Duration(1.0))
            self.tool_data[str(key)]=dict()
            self.tool_data[str(key)]['transform']   = a
            self.tool_data[str(key)]['matrix']      = AssemblyTools.to_homogeneous(a.transform.rotation, a.transform.translation)
            rospy.logerr("Added TCP entry for " + str(key))
            
        rospy.logerr("TCP position dictionary now contains: " + str(list(self.tool_data))+ ", selected tool publishing now: ")
        self.select_tool(self.activeTCP)

        self.surface_height = rospy.get_param('/task/assumed_starting_height') #Starting height assumption
        self.restart_height = rospy.get_param('/task/restart_height') #Height to restart
        # quit()

    def read_board_positions(self):
        """ Calculates pose of target hole relative to robot base frame.
        """
        temp_z_position_offset = 207 #Our robot is reading Z positions wrong on the pendant for some reason.
        taskPos = list(np.array(rospy.get_param('/environment_state/task_frame/position')))
        taskPos[2] = taskPos[2] + temp_z_position_offset
        taskOri = rospy.get_param('/environment_state/task_frame/orientation')
        holePos = list(np.array(rospy.get_param('/objects/'+self.target_hole+'/local_position')))
        holePos[2] = holePos[2] + temp_z_position_offset
        holeOri = rospy.get_param('/objects/'+self.target_hole+'/local_orientation')
        
        #Set up target hole pose
        tf_robot_to_task_board = AssemblyTools.get_tf_from_YAML(taskPos, taskOri, "base_link", "task_board")
        pose_task_board_to_hole = AssemblyTools.get_pose_from_YAML(holePos, holeOri, "base_link")
        self.target_hole_pose = tf2_geometry_msgs.do_transform_pose(pose_task_board_to_hole, tf_robot_to_task_board)
        # self.target_broadcaster = tf2_geometry_msgs.do_transform_pose(self.pose_task_board_to_hole, self.tf_robot_to_task_board)
        targetHoleTF = AssemblyTools.swap_pose_tf(self.target_hole_pose, "target_hole_position")
        self.reference_frames['target_hole_position'] = targetHoleTF
        self.send_reference_TFs()
        self._rate.sleep()
        # self._target_pub.publish(self.target_hole_pose)
        self.x_pos_offset = self.target_hole_pose.pose.position.x
        self.y_pos_offset = self.target_hole_pose.pose.position.y

    def read_peg_hole_dimensions(self):
        """Read peg and hole data from YAML configuration file.
        """
        peg_diameter         = rospy.get_param('/objects/'+self.target_peg+'/dimensions/diameter')/1000 #mm
        peg_tol_plus         = rospy.get_param('/objects/'+self.target_peg+'/tolerance/upper_tolerance')/1000
        peg_tol_minus        = rospy.get_param('/objects/'+self.target_peg+'/tolerance/lower_tolerance')/1000
        hole_diameter        = rospy.get_param('/objects/'+self.target_hole+'/dimensions/diameter')/1000 #mm
        hole_tol_plus        = rospy.get_param('/objects/'+self.target_hole+'/tolerance/upper_tolerance')/1000
        hole_tol_minus       = rospy.get_param('/objects/'+self.target_hole+'/tolerance/lower_tolerance')/1000    
        self.hole_depth      = rospy.get_param('/objects/'+self.target_peg+'/dimensions/min_insertion_depth')/1000
        
        #setup, run to calculate useful values based on params:
        self.clearance_max = hole_tol_plus - peg_tol_minus #calculate the total error zone;
        self.clearance_min = hole_tol_minus + peg_tol_plus #calculate minimum clearance;     =0
        self.clearance_avg = .5 * (self.clearance_max- self.clearance_min) #provisional calculation of "wiggle room"
        self.safe_clearance = (hole_diameter-peg_diameter + self.clearance_min)/2; # = .2 *radial* clearance i.e. on each side.
        # rospy.logerr("Peg is " + str(self.target_peg) + " and hole is " + str(self.target_hole))
        # rospy.logerr("Spiral pitch is gonna be " + str(self.safe_clearance) + "because that's min tolerance " + str(self.clearance_min) + " plus gap of " + str(hole_diameter-peg_diameter))
    
    def send_reference_TFs(self):
        if(self.reference_frames['tcp'].header.frame_id != ''):
            # print("Broadcasting tfs: " + str(self.reference_frames))
            self._rate.sleep()
            self.broadcaster.sendTransform(list(self.reference_frames.values()))
        else:
            rospy.logerr("Trying to publish headless TF!")
    @staticmethod
    def get_tf_from_YAML(pos, ori, base_frame, child_frame): #Returns the transform from base_frame to child_frame based on vector inputs
        """Reads a TF from config YAML.
        :param pos: (string) Param key for desired position parameter.
        :param ori: (string) Param key for desired orientation parameter.
        :param base_frame: (string) Base frame for output TF.
        :param child_frame:  (string) Child frame for output TF.
        :return: Geometry_Msgs.TransformStamped with linked parameters.
        """
        
        output_pose = AssemblyTools.get_pose_from_YAML(pos, ori, base_frame) #tf_task_board_to_hole
        output_tf = TransformStamped()
        output_tf.header = output_pose.header
        #output_tf.transform.translation = output_pose.pose.position
        [output_tf.transform.translation.x, output_tf.transform.translation.y, output_tf.transform.translation.z] = [output_pose.pose.position.x, output_pose.pose.position.y, output_pose.pose.position.z]
        output_tf.transform.rotation   = output_pose.pose.orientation
        output_tf.child_frame_id = child_frame
        
        return output_tf
    
    @staticmethod
    def get_pose_from_YAML(pos, ori, base_frame): #Returns the pose wrt base_frame based on vector inputs.
        """Reads a Pose from config YAML.
        :param pos: (string) Param key for desired position parameter.
        :param ori: (string) Param key for desired orientation parameter.
        :param base_frame: (string) Base frame for output pose.
        :param child_frame:  (string) Child frame for output pose.
        :return: Geometry_Msgs.PoseStamped with linked parameters.
        """
        
        #Inputs are in mm XYZ and degrees RPY
        #move to utils
        output_pose = PoseStamped() #tf_task_board_to_hole
        output_pose.header.stamp = rospy.get_rostime()
        output_pose.header.frame_id = base_frame
        tempQ = list(trfm.quaternion_from_euler(ori[0]*np.pi/180, ori[1]*np.pi/180, ori[2]*np.pi/180))
        output_pose.pose = Pose(Point(pos[0]/1000,pos[1]/1000,pos[2]/1000) , Quaternion(tempQ[0], tempQ[1], tempQ[2], tempQ[3]))
        
        return output_pose
    
    def select_tool(self, tool_name):
        """Sets activeTCP frame according to title of desired peg frame (tip, middle, etc.). This frame must be included in the YAML.
        :param tool_name: (string) Key in tool_data dictionary for desired frame.
        """
        # TODO: Make this a loop-run state to slowly slerp from one TCP to another using https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Slerp.html
        if(tool_name in list(self.tool_data)):
            self.activeTCP = tool_name
            self.reference_frames['tcp'] = self.tool_data[self.activeTCP]['transform']
            self.send_reference_TFs()
        else:
            rospy.logerr_throttle(2, "Tool selection key error! No key '" + tool_name + "' in tool dictionary.")

    def spiral_search_motion(self, frequency = .15, min_amplitude = .002, max_cycles = 62.83185):
        """Generates position, orientation offset vectors which describe a plane spiral about z; 
        Adds this offset to the current approach vector to create a searching pattern. Constants come from Init;
        x,y vector currently comes from x_ and y_pos_offset variables.
        """
        curr_time = rospy.get_rostime() - self._start_time
        curr_time_numpy = np.double(curr_time.to_sec())
        curr_amp = min_amplitude + self.safe_clearance * np.mod(2.0 * np.pi * frequency *curr_time_numpy, max_cycles);

        x_pos = curr_amp * np.cos(2.0 * np.pi * frequency * curr_time_numpy)

        y_pos = curr_amp * np.sin(2.0 * np.pi * frequency * curr_time_numpy)
        
        x_pos = x_pos + self.x_pos_offset
        y_pos = y_pos + self.y_pos_offset
        z_pos = self.current_pose.transform.translation.z #0.104 is the approximate height of the hole itself. TODO:Assume the part needs to be inserted here. Update once I know the real value

        pose_position = [x_pos, y_pos, z_pos]

        pose_orientation = [0, 1, 0, 0] # w, x, y, z

        return [pose_position, pose_orientation]

    def linear_search_position(self, direction_vector = [0,0,0], desired_orientation = [0, 1, 0, 0]):
        """Generates a command pose vector which causes the robot to hold a certain orientation
         and comply in z while maintaining the approach vector along x_ and y_pos_offset.
        :param direction_vector: (list of floats) vector directional offset from normal position. Causes constant motion in z.
        :param desired_orientation: (list of floats) quaternion parameters for orientation. 
        """
        pose_position = self.current_pose.transform.translation
        pose_position.x = self.x_pos_offset + direction_vector[0]
        pose_position.y = self.y_pos_offset + direction_vector[1]
        pose_position.z = pose_position.z + direction_vector[2]
        pose_orientation = desired_orientation
        return [[pose_position.x, pose_position.y, pose_position.z], pose_orientation]

    def full_compliance_position(self, direction_vector = [0,0,0], desired_orientation = [0, 1, 0, 0]):
        """Generates a command pose vector which causes the robot to hold a certain orientation
         and comply translationally in all directions.
        :param direction_vector: (list of floats) vector directional offset from normal position. Causes constant motion.
        :param desired_orientation: (list of floats) quaternion parameters for orientation. 
        """ 
        pose_position = self.current_pose.transform.translation
        pose_position.x = pose_position.x + direction_vector[0]
        pose_position.y = pose_position.y + direction_vector[1]
        pose_position.z = pose_position.z + direction_vector[2]
        pose_orientation = desired_orientation
        return [[pose_position.x, pose_position.y, pose_position.z], pose_orientation]

        #Load cell current data

    def callback_update_wrench(self, data: WrenchStamped):
        """Callback to update current wrench data whenever new data becomes available.
        """
        self.current_wrench = data

        # rospy.loginfo_once("Callback working! " + str(data))
    
    # def subtract_vector3s(self, vec1, vec2):
    #     newVector3 = Vector3(vec1.x - vec2.x, vec1.y - vec2.y, vec1.z - vec2.z)
    #     return newVector3

    def get_current_pos(self):
        """Read in current pose from robot base to activeTCP.        
        """
        transform = TransformStamped() #TODO: Check that this worked.
        # if(type(offset) == str):
        #     transform = self.tf_buffer.lookup_transform("base_link", self.activeTCP, rospy.Time(0), rospy.Duration(100.0))
        # else:
        if(self.activeTCP == "tool0"):
            transform = self.tf_buffer.lookup_transform("base_link", "tool0",
            rospy.Time(0), rospy.Duration(10.0))
        else:
            transform = self.tf_buffer.lookup_transform("base_link", self.tool_data[self.activeTCP]['transform'].child_frame_id,
            rospy.Time(0), rospy.Duration(10.0))
        return transform

    def get_command_wrench(self, vec = [0,0,0], ori = [0,0,0]):
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
        # self.check_controller(self.force_controller)
        # forces, torques = self.com_to_tcp(result[:3], result[3:], transform)
        # result_wrench = self.create_wrench(result[:3], result[3:])
        # result_wrench = self.create_wrench([7,0,0], [0,0,0])
        result_wrench = self.create_wrench(input_vec[:3], input_vec[3:])

        transform_world_to_gripper:TransformStamped = self.tf_buffer.lookup_transform('target_hole_position', 'tool0', rospy.Time(0), rospy.Duration(1.25))

        offset =Point( -1*self.tool_data[self.activeTCP]["transform"].transform.translation.x, -1*self.tool_data[self.activeTCP]["transform"].transform.translation.y, -1*(self.tool_data[self.activeTCP]["transform"].transform.translation.z - .05))

        transform_world_to_gripper.transform.translation = offset

        #Execute reinterpret-to-tcp and rotate-to-world simultaneously:
        result_wrench.wrench = AssemblyTools.transform_wrench(transform_world_to_gripper, result_wrench.wrench) #This works

        self._wrench_pub.publish(result_wrench)


    @staticmethod
    def list_from_quat(quat):
        return [quat.x, quat.y, quat.z, quat.w]

    @staticmethod
    def list_from_point(point):
        return [point.x, point.y, point.z]

    # def _publish_pose(self, position, orientation):
    def publish_pose(self, pose_stamped_vec):
        """Takes in vector representations of position 
        :param pose_stamped_vec: (list of floats) List of parameters for pose with x,y,z position and orientation quaternion
        """
        # Ensure controller is loaded
        # self.check_controller(self.controller_name)

        # Create poseStamped msg
        goal_pose = PoseStamped()

        # Set the position and orientation
        point = Point()
        quaternion = Quaternion()

        # point.x, point.y, point.z = position
        point.x, point.y, point.z = pose_stamped_vec[0][:]
        goal_pose.pose.position = point

        quaternion.w, quaternion.x, quaternion.y, quaternion.z  = pose_stamped_vec[1][:]
        goal_pose.pose.orientation = quaternion

        # Set header values
        goal_pose.header.stamp = rospy.get_rostime()
        goal_pose.header.frame_id = "base_link"
        


        if(self.activeTCP != "tool0"):
            #Convert pose in TCP coordinates to assign wrist "tool0" position for controller

            b_link = goal_pose.header.frame_id
            goal_matrix = AssemblyTools.to_homogeneous(goal_pose.pose.orientation, goal_pose.pose.position) #tf from base_link to tcp_goal = bTg
            backing_mx = trfm.inverse_matrix(self.tool_data[self.activeTCP]['matrix']) #tf from tcp_goal to wrist = gTw
            goal_matrix = np.dot(goal_matrix, backing_mx) #bTg * gTw = bTw
            goal_pose = AssemblyTools.matrix_to_pose(goal_matrix, b_link)
            
            # self._tool_offset_pub.publish(goal_pose)

            
        self._pose_pub.publish(goal_pose)

    @staticmethod
    def to_homogeneous(quat, point):
        """Takes a quaternion and msg.Point and outputs a homog. tf matrix.
        :param quat: (geometry_msgs.Quaternion) Orientation information.
        :param point: (geometry.msgs.Point) Position information.
        :return: (np.Array()) 4x4 Homogeneous transform matrix.
        """
        #TODO candidate for Utils 
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
        output.header.stamp = rospy.get_rostime()
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
        #Accomodation for input R_ab and P_ab
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
        return np.array([wrench.torque.x, wrench.torque.y, wrench.torque.z, wrench.force.x, wrench.force.y, wrench.force.z])
    
    @staticmethod
    def arrayToWrench(array: np.ndarray) -> np.ndarray:
        """Restructures output 1x6 mathematical array representation of a wrench into a wrench object.
        :param wrench: (np.Array) 1x6 numpy array 
        :return: (geometry_msgs.Wrench) Return wrench.
        """

        return Wrench(Point(*list(array[3:])), Point(*list(array[:3])))

    @staticmethod
    def transform_wrench(transform: TransformStamped, wrench: Wrench, invert:bool=False, log:bool=False) -> np.ndarray:
        """Transform a wrench object by the given transform object.
        :param transform: (geometry_msgs.TransformStamped) Transform to apply
        :param wrench: (geometry_msgs.Wrench) Wrench object to transform.
        :param invert: (bool) Whether to interpret the tansformation's inverse, i.e. transform "from child to parent" instead of "from parent to child"
        :return: (geometry.msgs.Wrench) changed wrench
        """

        matrix = AssemblyTools.to_homogeneous(transform.transform.rotation, transform.transform.translation)

        if(log):
            rospy.loginfo_throttle(2, Fore.RED + " Transform passed in is " + str(transform) + " and matrix passed in is \n" + str(matrix) + Style.RESET_ALL)
        
        if(invert):
            matrix = trfm.inverse_matrix(matrix)

        return AssemblyTools.transform_wrench_by_matrix(matrix, AssemblyTools.wrenchToArray(wrench))


    @staticmethod
    def transform_wrench_by_matrix(T_ab:np.ndarray, wrench:np.ndarray) -> np.ndarray:
        """Use the homogeneous transform (T_ab) to transform a given wrench using an adjoint transformation (see create_adjoint_representation).
        :param T_ab: (np.Array) 4x4 homogeneous transformation matrix representing frame 'b' relative to frame 'a'
        :param wrench: (np.Array) 6x1 representation of a wrench relative to frame 'a'. This should include forces and torques as np.array([torque, force])
        :return wrench_transformed: (np.Array) 6x1 representation of a wrench relative to frame 'b'. This should include forces and torques as np.array([torque, force])
        """

        Ad_T = AssemblyTools.create_adjoint_representation(T_ab)
        wrench_transformed = np.matmul(Ad_T.T, wrench)
        return AssemblyTools.arrayToWrench(wrench_transformed)

    @staticmethod
    def matrix_to_tf(input:np.ndarray, base_frame:String, child_frame:String):
        """Converts matrix back into a TF.
        :param input: (np.Array) 4x4 homogeneous transformation matrix
        :param base_frame: (string) base frame for new pose.
        :return: (geometry_msgs.TransformStamped) Transform based on input.
        """
        pose = AssemblyTools.matrix_to_pose(input, base_frame)
        output = AssemblyTools.swap_pose_tf(pose, child_frame)
        return output

    @staticmethod
    def swap_pose_tf(input:PoseStamped, child_frame:String) -> TransformStamped:
        """Swaps pose for tf and vice-versa.
        :param input: (geometry_msgs.PoseStamped or geometry_msgs.TransformStamped) Input data type.
        :param child_frame: (string) Child frame name if converting Pose to Transform.
        :return: (geometry_msgs.TransformStamped or geometry_msgs.PoseStamped) Output data, of the other type from input.
        """
        if('PoseStamped' in str(type(input))):
            output = TransformStamped()
            output.header = input.header
            # output.transform = input.pose
            [output.transform.translation.x, output.transform.translation.y, output.transform.translation.z] = [input.pose.position.x, input.pose.position.y, input.pose.position.z]
            output.transform.rotation   = input.pose.orientation  

            output.child_frame_id = child_frame
            return output
        else:
            if('TransformStamped' in str(type(input))):
                output = PoseStamped()
                output.header = input.header
                output.pose = input.transform
                return output
        rospy.logerr("Invalid input to swap_pose_tf !!!")

    def create_wrench(self, force:list, torque:list) -> WrenchStamped:
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

        wrench_stamped.header.stamp = rospy.get_rostime()
        wrench_stamped.header.frame_id = "base_link"
        # self._seq+=1

        wrench_stamped.wrench = wrench

        return wrench_stamped

    def update_average_wrench(self) -> None:
        """Create a very simple moving average of the incoming wrench readings and store it as self.average.wrench.
        """

        
        self._average_wrench_gripper = self.filters.average_wrench(self.current_wrench.wrench)

        #Get current angle from gripper to hole:
        transform_world_rotation:TransformStamped = self.tf_buffer.lookup_transform('tool0', 'target_hole_position', rospy.Time(0), rospy.Duration(1.25))
        #We want to rotate this only, not reinterpret F/T components.
        #We reinterpret based on the position of the TCP (but ignore the relative rotation). In addition, the wrench is internally measured at the load cell and has a built-in transformation to tool0 which is 5cm forward. We have to undo that transformation to get accurate transformation.                       
        offset =Point(self.tool_data[self.activeTCP]["transform"].transform.translation.x, self.tool_data[self.activeTCP]["transform"].transform.translation.y, self.tool_data[self.activeTCP]["transform"].transform.translation.z - .05)

            
        transform_world_rotation.transform.translation = offset

        #Execute reinterpret-to-tcp and rotate-to-world simultaneously:
        self._average_wrench_world = AssemblyTools.transform_wrench(transform_world_rotation, self._average_wrench_gripper) #This works

        #Output the wrench for debug visualization
        guy = self.create_wrench([0,0,0], [0,0,0])
        guy.wrench = self._average_wrench_world
        # guy.header.frame_id = "tool0"
        guy.header.frame_id = "target_hole_position"
        # guy.header.frame_id =  self.reference_frames['tcp'].child_frame_id
        self._adj_wrench_pub.publish(guy)    


    def update_avg_speed(self) -> None:
        """Updates a simple moving average of robot tcp speed in mm/s. A speed is calculated from the difference between a
         previous pose (.1 s in the past) and the current pose; this speed is filtered and stored as self.average_speed.
        """
        curr_time = rospy.get_rostime() - self._start_time
        if(curr_time.to_sec() > rospy.Duration(.5).to_sec()):
            try:
                earlierPosition = self.tf_buffer.lookup_transform("base_link", self.tool_data[self.activeTCP]['transform'].child_frame_id, 
                    rospy.Time.now() - rospy.Duration(.1), rospy.Duration(2.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                raise
            #Speed Diff: distance moved / time between poses
            positionDiff = self.as_array(self.current_pose.transform.translation) - self.as_array(earlierPosition.transform.translation)
            timeDiff = ((self.current_pose.header.stamp) - (earlierPosition.header.stamp)).to_sec()
            if(timeDiff > 0.0): #Update only if we're using a new pose; also, avoid divide by zero
                speedDiff = positionDiff / timeDiff
                #Moving averate weighted toward old speed; response is independent of rate selected.
                # self.average_speed = self.average_speed * (1-10/self._rate_selected) + speedDiff * (10/self._rate_selected)
                # rospy.logwarn_throttle(2.0, "Speed is currently about " + str(speedDiff))
                self.average_speed = self.filters.average_speed(speedDiff)
        else:
            rospy.logwarn_throttle(1.0, "Too early to report past time!" + str(curr_time.to_sec()))
    
    def publish_plotted_values(self) -> None:
        """Publishes critical data for plotting node to process.
        """
        self.avg_wrench_pub.publish(self._average_wrench_world)

        self.avg_speed_pub.publish(Point(self.average_speed[0], self.average_speed[1],self.average_speed[2]))

        self.rel_position_pub.publish(self.current_pose.transform.translation)
        
        # Send a dictionary as plain text to expose some additional info
        status_dict = dict({('state', self.state), ('tcp_name', str(self.tool_data[self.activeTCP]['transform'].child_frame_id) )})
        if(self.surface_height != 0.0):
            # If we have located the work surface
            status_dict['surface_height']=str(self.surface_height)
        self.status_pub.publish(str(status_dict))

    def as_array(self, vec:Point) -> np.ndarray:
        """Takes a Point and returns a Numpy array.
        :param vec: (geometry_msgs.Point) Vector in serialized ROS format.
        :return: (numpy.Array) Vector in 3x1 numpy array format.
        """
        #insist that we get a 1D array returned
        return np.array([vec.x, vec.y, vec.z]).reshape(-1,)
    
    #See if the force/speed (any vector) is within a 3-d bound. Technically, it is a box, with sqrt(2)*bound okay at diagonals.
    def vectorRegionCompare_symmetrical(self, input:np.ndarray, bounds_max:list) -> bool:
        """See ``vectorRegionCompare``_. Compares an input to boundaries element-wise. Essentially checks whether a vector
         is within a rectangular region. This version assumes min values to be the negative of max values.
        :param input: (list of floats) x,y,z of a vector to check.
        :param bounds_max: (list of floats) x,y,z max value of each element.
        :return: (bool) Whether the vector falls within the region.
        """

        #initialize a minimum list
        bounds_min = [0,0,0] 
        #Each min value is the negative of the max value
        #Create bounds_min to be the negative of bounds_max. symmetrical, duh....
        bounds_min[0] = bounds_max[0] * -1.0
        bounds_min[1] = bounds_max[1] * -1.0
        bounds_min[2] = bounds_max[2] * -1.0
        return self.vectorRegionCompare(input, bounds_max, bounds_min)
    
    # bounds_max and bounds_min let you set a range for each dimension. 
    #This just compares if you are in the cube described above. 
    def vectorRegionCompare(self, input:list, bounds_max:list, bounds_min:list) -> bool:
        """.. vectorRegionCompare Compares an input to boundaries element-wise. Essentially checks whether a vector is within a rectangular region.
        :param input: (list of floats) x,y,z of a vector to check.
        :param bounds_max: (list of floats) x,y,z max value of each element.
        :param bounds_min: (list of floats) x,y,z min value of each element.
        :return: (bool) Whether the vector falls within the region. 
        """
        #Simply compares abs. val.s of input's elements to a vector of maximums and returns whether it exceeds
        #if(symmetrical):
        #    bounds_min[0], bounds_min[1], bounds_min[2] = bounds_max[0] * -1, bounds_max[1] * -1, bounds_max[2] * -1
        #TODO - convert to a process of numpy arrays! They process way faster because that library is written in C++
        #Note - actually Numpy's allclose() method may be perfect here.

        # rospy.logerr("Bounds and stuff: " +str(input) + str(bounds_max) + str(bounds_min))
        if( bounds_max[0] >= input[0] >= bounds_min[0]):
            if( bounds_max[1] >= input[1] >= bounds_min[1]):
                if( bounds_max[2] >= input[2] >= bounds_min[2]):
                    return True
        return False

    def checkIfStatic(self, maxSpeeds:np.ndarray)->bool:
        res = np.allclose(abs(self.average_speed), np.zeros(3), atol=abs( maxSpeeds))
        
        return res
        # return self.vectorRegionCompare_symmetrical(self.average_speed, maxSpeeds)

    def checkIfColliding(self, commandedForce:np.ndarray, deadzoneRadius:list = [4,4,3], relativeScaling:float = .1)->bool:
        """Checks if  an equal and opposite reaction force is stopping acceleration in all directions - this would indicate there is a static obstacle in collision with the tcp.
        """
        force = self.as_array(self._average_wrench_world.force).reshape(3)
        res = np.allclose(force, -1*commandedForce, atol = deadzoneRadius, rtol = relativeScaling )
        
        # rospy.loginfo_throttle(1,Fore.BLUE +  "Collision checking force " + str(force) + " against command " + str(commandedForce*-1) + ' with a result of ' + str(res) + " and the difference is " + str(force + commandedForce) + Style.RESET_ALL)

        return res
        # if(type(lowerThresholds) == type(None)):
        #     lowerThresholds = -1 * upperThresholds
        # return self.vectorRegionCompare(self.as_array(self._average_wrench_world.force)-commandedForce, upperThresholds, lowerThresholds)

    #TODO: Make the parameters of function part of the constructor or something...
    def force_cap_check(self, danger_force=[45, 45, 45], danger_transverse_force=[3.5, 3.5, 3.5], warning_force=[25, 25, 25], warning_transverse_force=[2, 2, 2]):
        """Checks whether any forces or torques are dangerously high. There are two levels of response:
            *Elevated levels of force cause this program to pause for 1s. If forces remain high after pause, 
            the system will enter a freewheeling state
            *Dangerously high forces will kill this program immediately to prevent damage.
        :return: (Bool) True if all is safe; False if a warning stop is requested.
        """
        #Calculate acceptable torque from transverse forces
        radius = np.linalg.norm(self.as_array(self.tool_data[self.activeTCP]['transform'].transform.translation))
        #Set a minimum radius to always permit some torque
        radius = max(3, radius)
        rospy.loginfo_once("For TCP " + self.activeTCP + " moment arm is coming out to " + str(radius))
        warning_torque=[warning_transverse_force[a]*radius for a in range(3)]
        danger_torque=[danger_transverse_force[b]*radius for b in range(3)]
        rospy.loginfo_once("So torques are limited to  " + str(warning_torque) + str(danger_torque))

        if(not (self.vectorRegionCompare_symmetrical(self.as_array(self.current_wrench.wrench.force), danger_force)
            and self.vectorRegionCompare_symmetrical(self.as_array(self.current_wrench.wrench.torque), danger_torque))):
                rospy.logerr("*Very* high force/torque detected! " + str(self.current_wrench.wrench))
                rospy.logerr("Killing program.")
                quit() # kills the program. Since the node is required, it kills the ROS application.
                return False
        if(self.vectorRegionCompare_symmetrical(self.as_array(self.current_wrench.wrench.force), warning_force)):
            if(self.vectorRegionCompare_symmetrical(self.as_array(self.current_wrench.wrench.torque), warning_torque)):
                return True
        rospy.logerr("High force/torque detected! " + str(self.current_wrench.wrench))
        if(self.highForceWarning):
            self.highForceWarning = False
            return False
        else:   
            rospy.logerr("Sleeping for 1s to damp oscillations...")
            self.highForceWarning = True
            rospy.sleep(1) #Want the system to stop for a second in hopes that it prevents higher forces/torques. May not be helping.
        return True
        
class AssemblyFilters():
    """Averages a signal based on a history log of previous values. Window size is normallized to different
    frequency values using _rate_selected; window should be for 100hz cycle time.
    """

    def __init__(self, window = 15, rate_selected=100):


        #Simple Moving Average Parameters
        self._rate_selected = rate_selected
        self._buffer_window = dict()
        self._buffer_window["wrench"] = window # should tie to self._rate_selected = 1/Hz since this variable is the rate of ROS commands
        self._data_buffer = dict()
        # self._moving_avg_data = np. #Empty to start. make larger than we need since np is contiguous memory. Will ignore NaN values.
        # self._data_buffer = np.empty(self._buffer_window)
        # self.avg_it = 0#iterator for allocating the first window in the moving average calculation
        # self._data_buffer = np.zeros(self._buffer_window)
        # self._moving_avg_data = [] #Empty to start
    
    def average_wrench(self, input)-> np.ndarray:
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
            vals[k] = self.simple_moving_average(v, 15, key=name+'_'+k)
        return vals

    def point_to_dict(self, input):
        return {"x": input.x, "y":input.y, "z":input.z}

    def dict_to_point(self, input):
        return Point(input["x"], input["y"], input["z"])

    def simple_moving_average(self, new_data_point, window=None, key="wrench"):

        if not key in self._data_buffer:
            self._data_buffer[key] = np.array([])
            self._buffer_window[key] = window
        window =  int(np.floor(self._buffer_window[key] * self._rate_selected/100)) #Unless new input provided, use class member
        #Fill up the first window while returning current value, else calculate moving average using constant window
        if len(self._data_buffer[key]) < window:
            self._data_buffer[key] = np.append(self._data_buffer[key], new_data_point)
            avg = self.calc_moving_average(self._data_buffer[key], len(self._data_buffer[key]))
        else:
            self._data_buffer[key] = np.append(self._data_buffer[key], new_data_point) #append new datapoint to the end
            self._data_buffer[key] = np.delete(self._data_buffer[key], 0) #pop the first element
            avg = self.calc_moving_average(self._data_buffer[key], window)
        
        return avg
        
    def calc_moving_average(self, buffered_data, w): #w is the window
        return np.convolve(buffered_data, np.ones(w), 'valid') / w

if __name__ == '__main__':
    rospy.init_node("demo_assembly_application_compliance")
    
    
