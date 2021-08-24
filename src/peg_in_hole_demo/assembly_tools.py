#!/usr/bin/env python

# Imports for ros
from inspect import EndOfBlock
from operator import truediv
import rospy
# import tf
import numpy as np
import matplotlib.pyplot as plt
from rospkg import RosPack
from geometry_msgs.msg import WrenchStamped, Wrench, TransformStamped, PoseStamped, Pose, Point, Quaternion, Vector3, Transform
from rospy.core import configure_logging

from sensor_msgs.msg import JointState
# from assembly_ros.srv import ExecuteStart, ExecuteRestart, ExecuteStop
from controller_manager_msgs.srv import SwitchController, LoadController, ListControllers

import tf2_ros
# import tf2
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler

from threading import Lock


#State names    
IDLE_STATE           = 'idle'
CHECK_FEEDBACK_STATE = 'checking load cell feedback'
APPROACH_STATE       = 'approaching hole surface'
FIND_HOLE_STATE      = 'finding hole'
INSERTING_PEG_STATE  = 'inserting peg'
COMPLETION_STATE     = 'completed insertion'
SAFETY_RETRACT_STATE = 'retracting to safety' 

#Trigger names
CHECK_FEEDBACK_TRIGGER     = 'check loadcell feedback'
APPROACH_SURFACE_TRIGGER   = 'start approach'
FIND_HOLE_TRIGGER          = 'surface found'
INSERT_PEG_TRIGGER         = 'hole found'
ASSEMBLY_COMPLETED_TRIGGER = 'assembly completed'
SAFETY_RETRACTION_TRIGGER  = 'retract to safety'


class AssemblyTools():

    def __init__(self, ROS_rate, start_time):

        #Needed to get current pose of the robot
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self._rate_selected = ROS_rate
        self._rate = rospy.Rate(self._rate_selected) #setup for sleeping in hz
        self._seq = 0
        self._start_time = start_time #for _spiral_search_basic_force_control and _spiral_search_basic_compliance_control
        
        #Spiral parameters
        self._freq = np.double(0.15) #Hz frequency in _spiral_search_basic_force_control
        self._amp  = np.double(10.0)  #Newton amplitude in _spiral_search_basic_force_control
        self._first_wrench = self._create_wrench([0,0,0], [0,0,0])
        self._freq_c = np.double(0.15) #Hz frequency in _spiral_search_basic_compliance_control
        self._amp_c  = np.double(.002)  #meters amplitude in _spiral_search_basic_compliance_control
        self._amp_limit_c = 2 * np.pi * 10 #search number of radii distance outward

        #job parameters moved in from the peg_in_hole_params.yaml file
        target_peg = 'peg_10mm'
        target_hole = 'hole_10mm'
        temp_z_position_offset = 207/1000 #Our robot is reading Z positions wrong on the pendant for some reason.
        taskPos = list(np.array(rospy.get_param('/environment_state/task_frame/position'))/1000)
        taskPos[2] = taskPos[2] + temp_z_position_offset
        taskOri = rospy.get_param('/environment_state/task_frame/orientation')
        holePos = list(np.array(rospy.get_param('/objects/'+target_hole+'/local_position'))/1000)
        holePos[2] = holePos[2] + temp_z_position_offset
        holeOri = rospy.get_param('/objects/'+target_hole+'/local_orientation')
        
        self.tf_robot_to_task_board = TransformStamped() #tf_task_board_to_hole
        self.tf_robot_to_task_board.header.stamp = rospy.get_rostime()
        self.tf_robot_to_task_board.header.frame_id = "base_link"
        self.tf_robot_to_task_board.child_frame_id = "task_board"
        tempQ = list(quaternion_from_euler(taskOri[0]*np.pi/180, taskOri[1]*np.pi/180, taskOri[2]*np.pi/180))
        self.tf_robot_to_task_board.transform = Transform(Point(taskPos[0],taskPos[1],taskPos[2]) , Quaternion(tempQ[0], tempQ[1], tempQ[2], tempQ[3]))
        
        self.pose_task_board_to_hole = PoseStamped() #tf_task_board_to_hole
        self.pose_task_board_to_hole.header.stamp = rospy.get_rostime()
        self.pose_task_board_to_hole.header.frame_id = "task_board"
        tempQ = list(quaternion_from_euler(holeOri[0]*np.pi/180, holeOri[1]*np.pi/180, holeOri[2]*np.pi/180))
        self.pose_task_board_to_hole.pose = Pose(Point(holePos[0],holePos[1],holePos[2]), Quaternion(tempQ[0], tempQ[1], tempQ[2], tempQ[3]))
        
        self.target_hole_pose = tf2_geometry_msgs.do_transform_pose(self.pose_task_board_to_hole, self.tf_robot_to_task_board)

        #rospy.logerr("Hole Pose: " + str(self.target_hole_pose))
        self._target_pub.publish(self.target_hole_pose)
        self.x_pos_offset = self.target_hole_pose.pose.position.x
        self.y_pos_offset = self.target_hole_pose.pose.position.y
        
        peg_diameter     = rospy.get_param('/objects/'+target_peg+'/dimensions/diameter')/1000 #mm
        peg_tol_plus     = rospy.get_param('/objects/'+target_peg+'/tolerance/upper_tolerance')/1000
        peg_tol_minus    = rospy.get_param('/objects/'+target_peg+'/tolerance/lower_tolerance')/1000

        hole_diameter    = rospy.get_param('/objects/'+target_hole+'/dimensions/diameter')/1000 #mm
        hole_tol_plus    = rospy.get_param('/objects/'+target_hole+'/tolerance/upper_tolerance')/1000
        hole_tol_minus   = rospy.get_param('/objects/'+target_hole+'/tolerance/lower_tolerance')/1000    
        self.hole_depth  = rospy.get_param('/objects/'+target_peg+'/dimensions/min_insertion_depth')/1000
        #self.hole_depth = rospy.get_param('/objects/peg/dimensions/length', )
        #self.hole_depth = .0075 #we need to insert at least this far before it will consider if it's inserted
        
        #loop parameters
        self.curr_time = rospy.get_rostime() - self._start_time
        self.curr_time_numpy = np.double(self.curr_time.to_sec())
        self.wrench_vec  = self._get_command_wrench([0,0,0])
        self.next_trigger = '' #Empty to start. Each callback should decide what next trigger to implement in the main loop

        self.current_pose = self._get_current_pos()
        self.pose_vec = self._full_compliance_position()
        self.current_wrench = self._first_wrench
        self._average_wrench = self._first_wrench.wrench 
        self._bias_wrench = self._first_wrench.wrench #Calculated to remove the steady-state error from wrench readings. 
        #TODO - subtract bias_wrench from the "current wrench" callback; Tried it but performance was unstable.
        self.average_speed = np.array([0.0,0.0,0.0])

        #Simple Moving Average Parameters
        self._buffer_window = self._rate_selected #self._rate_selected = 1/Hz since this variable is the rate of ROS commands
        self._data_buffer = []
        # self._moving_avg_data = np. #Empty to start. make larger than we need since np is contiguous memory. Will ignore NaN values.
        # self._data_buffer = np.empty(self._buffer_window)
        # self.avg_it = 0#iterator for allocating the first window in the moving average calculation
        # self._data_buffer = np.zeros(self._buffer_window)
        # self._moving_avg_data = [] #Empty to start

        #setup, run to calculate useful values based on params:
        self.clearance_max = hole_tol_plus - peg_tol_minus #calculate the total error zone;
        self.clearance_min = hole_tol_minus + peg_tol_plus #calculate minimum clearance;     =0
        self.clearance_avg = .5 * (self.clearance_max- self.clearance_min) #provisional calculation of "wiggle room"
        self.safe_clearance = (hole_diameter-peg_diameter + self.clearance_min)/2; # = .2 *radial* clearance i.e. on each side.

        #setup, run to calculate useful values based on params:
        self.clearance_max = hole_tol_plus - peg_tol_minus #calculate the total error zone;
        self.clearance_min = hole_tol_minus + peg_tol_plus #calculate minimum clearance;     =0
        self.clearance_avg = .5 * (self.clearance_max- self.clearance_min) #provisional calculation of "wiggle room"
        self.safe_clearance = (hole_diameter-peg_diameter + self.clearance_min)/2; # = .2 *radial* clearance i.e. on each side.
        rospy.logerr("Peg is " + str(target_peg) + " and hole is " + str(target_hole))
        rospy.logerr("Spiral pitch is gonna be " + str(self.safe_clearance) + "because that's min tolerance " + str(self.clearance_min) + " plus gap of " + str(hole_diameter-peg_diameter))

        self.highForceWarning = False
        self.surface_height = None
        self.restart_height = .1
        self.collision_confidence = 0;
        

    def _spiral_search_basic_compliance_control(self):
        #Generate position, orientation vectors which describe a plane spiral about z; conform to the current z position. 
        curr_time = rospy.get_rostime() - self._start_time
        curr_time_numpy = np.double(curr_time.to_sec())
        curr_amp = self._amp_c + self.safe_clearance * np.mod(2.0 * np.pi * self._freq_c *curr_time_numpy, self._amp_limit_c);

        # x_pos_offset = 0.88 #TODO:Assume the part needs to be inserted here at the offset. Fix with real value later
        # y_pos_offset = 0.550 #TODO:Assume the part needs to be inserted here at the offset. Fix with real value later
        
        # self._amp_c = self._amp_c * (curr_time_numpy * 0.001 * curr_time_numpy+ 1)

        x_pos = curr_amp * np.cos(2.0 * np.pi * self._freq_c *curr_time_numpy)
        x_pos = x_pos + self.x_pos_offset

        y_pos = curr_amp * np.sin(2.0 * np.pi * self._freq_c *curr_time_numpy)
        y_pos = y_pos + self.y_pos_offset

        # z_pos = 0.2 #0.104 is the approximate height of the hole itself. TODO:Assume the part needs to be inserted here. Update once I know the real value 
        z_pos = self.current_pose.transform.translation.z #0.104 is the approximate height of the hole itself. TODO:Assume the part needs to be inserted here. Update once I know the real value

        pose_position = [x_pos, y_pos, z_pos]

        pose_orientation = [0, 1, 0, 0] # w, x, y, z, TODO: Fix such that Jerit's code is not assumed correct. Is this right?

        return [pose_position, pose_orientation]
    def _linear_search_position(self, direction_vector = [0,0,0], desired_orientation = [0, 1, 0, 0]):
        #makes a command to simply stay still in a certain orientation
        pose_position = self.current_pose.transform.translation
        pose_position.x = self.x_pos_offset + direction_vector[0]
        pose_position.y = self.y_pos_offset + direction_vector[1]
        pose_position.z = pose_position.z + direction_vector[2]
        pose_orientation = desired_orientation
        return [[pose_position.x, pose_position.y, pose_position.z], pose_orientation]

        #function used for parking the robot in neutral. The pose_position needs to be published elsewhere.
    def _full_compliance_position(self, direction_vector = [0,0,0], desired_orientation = [0, 1, 0, 0]):
        #makes a command to simply stay still in a certain orientation
        pose_position = self.current_pose.transform.translation
        pose_position.x = pose_position.x + direction_vector[0]
        pose_position.y = pose_position.y + direction_vector[1]
        pose_position.z = pose_position.z + direction_vector[2]
        pose_orientation = desired_orientation
        return [[pose_position.x, pose_position.y, pose_position.z], pose_orientation]

        #Load cell current data

    # Defines the state machine's next trigger it should execute 
    def post_action(self, trigger_name):
        return [trigger_name, True]


    def _subtract_vector3s(self, vec1, vec2):
        newVector3 = Vector3(vec1.x - vec2.x, vec1.y - vec2.y, vec1.z - vec2.z)
        return newVector3

    def _get_current_pos(self):

        transform = self.tf_buffer.lookup_transform("base_link", "tool0", rospy.Time(0), rospy.Duration(100.0))
        return transform

    #Convert normal math to ROS wrench. 5 is the default downward commanded force.
    def _get_command_wrench(self, vec = [0,0,5]):
        # curr_time = rospy.get_rostime() - self._start_time
        # curr_time_numpy = np.double(curr_time.to_sec())

        # x_f = self._amp * np.cos(2.0 * np.pi * self._freq *curr_time_numpy)
        # y_f = self._amp * np.sin(2.0 * np.pi * self._freq *curr_time_numpy)
        x_f = vec[0]
        y_f = vec[1]
        z_f = vec[2] #apply constant downward force

        return [x_f, y_f, z_f, 0, 0, 0]

    def _calibrate_force_zero(self):
        curr_time = rospy.get_rostime() - self._start_time
        curr_time_numpy = np.double(curr_time.to_sec())

    def _create_wrench(self, force, torque):
        wrench_stamped = WrenchStamped()
        wrench = Wrench()

        # create wrench
        wrench.force.x, wrench.force.y, wrench.force.z = force
        wrench.torque.x, wrench.torque.y, wrench.torque.z = torque

        # create header
        wrench_stamped.header.seq = self._seq

        wrench_stamped.header.stamp = rospy.get_rostime()
        wrench_stamped.header.frame_id = "base_link"
        self._seq+=1

        wrench_stamped.wrench = wrench

        return wrench_stamped
    
    # Not sure if working correctly. 9 abnd 1 are weights. Does not work well when you change selected rate variable.
    #The faster you run the function, the faster it responds to changes in the force/torque. 
    #Trying to create a slow moving average of the force and torque data. Maybe get numpy/scipi low-pass implementation
    def _update_average_wrench(self):
        #get a very simple average of wrench reading
        #self._average_wrench = self._weighted_average_wrenches(self._average_wrench, 9, self.current_wrench.wrench, 1)
        self._average_wrench = self._weighted_average_wrenches(self._average_wrench, 9, self.current_wrench.wrench, 1)
        #rospy.logwarn_throttle(.5, "Updating wrench toward " + str(self.current_wrench.wrench.force))

    def _weighted_average_wrenches(self, wrench1, scale1, wrench2, scale2):
        newForce = (self._as_array(wrench1.force) * scale1 + self._as_array(wrench2.force) * scale2) * 1/(scale1 + scale2)
        newTorque = (self._as_array(wrench1.torque) * scale1 + self._as_array(wrench2.torque) * scale2) * 1/(scale1 + scale2)
        return self._create_wrench([newForce[0], newForce[1], newForce[2]], [newTorque[0], newTorque[1], newTorque[2]]).wrench
            
    #Go back in time a 1/10 of a second (0.1). This function is averaging the speed instead of the wrench data. 
    #This function currently works though. 
    #instantaneious speed with a 0.1 second time step.
    #Not tested, but it is used and seems to work fine. 
    def _update_avg_speed(self):
        curr_time = rospy.get_rostime() - self._start_time
        if(curr_time.to_sec() > rospy.Duration(.5).to_sec()):
            #rospy.logwarn("Averageing speed! Time:" + str(curr_time.to_sec()))
            #currentPosition = self._get_current_pos().transform.translation;
            #earlierPosition = self.tf_buffer.lookup_transform("base_link", "tool0", rospy.Time.now() - rospy.Duration(.1), rospy.Duration(100.0))
            try:
                #earlierPosition = self.tf_buffer.lookup_transform("base_link", "tool0", rospy.Time(0), rospy.Duration(2.0))
                earlierPosition = self.tf_buffer.lookup_transform("base_link", "tool0", rospy.Time.now() - rospy.Duration(.1), rospy.Duration(2.0))
                # earlierPosition = self.tf_buffer.lookup_transform_full(
                #     "tool0",
                #     (rospy.Time.now() - rospy.Duration(.1)),
                #     "base_link",
                #     (rospy.Time.now() - rospy.Duration(.1)),
                #     "base_link",
                #     rospy.Duration(1)
                # )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                raise
            #Speed Diff: distance moved / time between poses
            positionDiff = self._as_array(self.current_pose.transform.translation) - self._as_array(earlierPosition.transform.translation)
            timeDiff = ((self.current_pose.header.stamp) - (earlierPosition.header.stamp)).to_sec()
            if(timeDiff > 0.0): #Update only if we're using a new pose; also, avoid divide by zero
                speedDiff = positionDiff / timeDiff
                #Moving averate weighted toward old speed; response is now independent of rate selected.
                self.average_speed = self.average_speed * (1-10/self._rate_selected) + speedDiff * (10/self._rate_selected)
            #rospy.logwarn("Speed average: " + str(self.average_speed) )
        else:
            rospy.logwarn_throttle(1.0, "Too early to report past time!" + str(curr_time.to_sec()))
    @staticmethod
    def _as_array(vec):
        return np.array([vec.x, vec.y, vec.z])
    
    #See if the force/speed (any vector) is within a 3-d bound. Technically, it is a box, with sqrt(2)*bound okay at diagonals.
    def _vectorRegionCompare_symmetrical(self, input, bounds_max):
        #initialize a minimum list
        bounds_min = [0,0,0] 
        #Each min value is the negative of the max value
        #Create bounds_min to be the negative of bounds_max. symmetrical, duh....
        bounds_min[0] = bounds_max[0] * -1.0
        bounds_min[1] = bounds_max[1] * -1.0
        bounds_min[2] = bounds_max[2] * -1.0
        return self._vectorRegionCompare(input, bounds_max, bounds_min)
    
    # bounds_max and bounds_min let you set a range for each dimension. 
    #This just compares if you are in the cube described above. 
    def _vectorRegionCompare(self, input, bounds_max, bounds_min):
        #Simply compares abs. val.s of input's elements to a vector of maximums and returns whether it exceeds
        #if(symmetrical):
        #    bounds_min[0], bounds_min[1], bounds_min[2] = bounds_max[0] * -1, bounds_max[1] * -1, bounds_max[2] * -1
        #TODO - convert to a process of numpy arrays! They process way faster because that library is written in C++
        if( bounds_max[0] >= input[0] >= bounds_min[0]):
            if( bounds_max[1] >= input[1] >= bounds_min[1]):
                if( bounds_max[2] >= input[2] >= bounds_min[2]):
                    #rospy.logwarn(.5, "_______________ping!________________")
                    return True
        return False

    #TODO: Make the parameters of function part of the constructor or something...
    def _force_cap_check(self):
        #Checks if any forces or torques are dangerously high.

        if(not (self._vectorRegionCompare_symmetrical(self._as_array(self.current_wrench.wrench.force), [45, 45, 45])
            and self._vectorRegionCompare_symmetrical(self._as_array(self.current_wrench.wrench.torque), [3.5, 3.5, 3.5]))):
                rospy.logerr("*Very* high force/torque detected! " + str(self.current_wrench.wrench))
                rospy.logerr("Killing program.")
                quit() # kills the program. Since the node is required, it kills the ROS application.
                return False
        if(self._vectorRegionCompare_symmetrical(self._as_array(self.current_wrench.wrench.force), [25, 25, 25])):
            if(self._vectorRegionCompare_symmetrical(self._as_array(self.current_wrench.wrench.torque), [2, 2, 2])):
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

    def __init__(self):
        #Simple Moving Average Parameters
        self._buffer_window = self._rate_selected #self._rate_selected = 1/Hz since this variable is the rate of ROS commands
        self._data_buffer = []
        # self._moving_avg_data = np. #Empty to start. make larger than we need since np is contiguous memory. Will ignore NaN values.
        # self._data_buffer = np.empty(self._buffer_window)
        # self.avg_it = 0#iterator for allocating the first window in the moving average calculation
        # self._data_buffer = np.zeros(self._buffer_window)
        # self._moving_avg_data = [] #Empty to start

    def _simple_moving_average(self, new_data_point, window=None):
        if window == None:
            window =  self._buffer_window #Unless new input provided, use class member

        #Fill up the first window while returning current value, else calculate moving average using constant window
        if len(self._data_buffer) < window:
            self._data_buffer = np.append(self.data_buffer, new_data_point)
            avg = self._calc_moving_average(self._data_buffer, len(self._data_buffer))
        else:
            self._data_buffer = np.append(self._data_buffer, new_data_point) #append new datapoint to the end
            self.data_buffer = np.delete(self.data_buffer, 0) #pop the first element
            avg = self._calc_moving_average(self._data_buffer, window)
        
        return avg
        
    def _calc_moving_average(buffered_data, w): #w is the window
        return np.convolve(buffered_data, np.ones(w), 'valid') / w


if __name__ == '__main__':
    rospy.init_node("demo_assembly_application_compliance")
    
    