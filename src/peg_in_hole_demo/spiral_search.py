#!/usr/bin/env python

#UR IP Address is now 175.31.1.137
#Computer has to be 175.31.1.150

# Imports for ros
# from _typeshed import StrPath
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

import peg_in_hole_demo.assembly_state_emitter as aes

from transitions import Machine

# class PegInHoleStateMachine():
#     states = ['approaching hole surface', 'finding hole', 'inserting peg', 'completed insertion']

#     transitions = [
#         {'trigger':'surface found', 'source':'approaching hole surface', 'dest':'finding hole' }
#     ]

#     def __init__(self):

#         self.state_machine = Machine(model=self, states=PegInHoleStateMachine.states, transitions=PegInHoleStateMachine.transitions, initial='approaching hole surface')



#State names
IDLE_STATE           = 'idle state'
CHECK_FEEDBACK_STATE = 'checking load cell feedback'
APPROACH_STATE       = 'approaching hole surface'
FIND_HOLE_STATE      = 'finding hole'
INSERTING_PEG_STATE  = 'inserting peg'
COMPLETION_STATE     = 'completed insertion'
SAFETY_RETRACT_STATE = 'retracing to safety' 

#Trigger names
CHECK_FEEDBACK_TRIGGER     = 'check loadcell feedback'
START_APPROACH_TRIGGER     = 'start approach'
SURFACE_FOUND_TRIGGER      = 'surface found'
HOLE_FOUND_TRIGGER         = 'hole found'
ASSEMBLY_COMPLETED_TRIGGER = 'assembly completed'
SAFETY_RETRACTION_TRIGGER  = 'retract to safety'

class PegInHoleNodeCompliance(Machine):



    def __init__(self):
        


        states = [
            IDLE_STATE,
            CHECK_FEEDBACK_STATE,
            APPROACH_STATE, 
            FIND_HOLE_STATE, 
            INSERTING_PEG_STATE, 
            COMPLETION_STATE, 
            SAFETY_RETRACT_STATE
        ]

        transitions = [
            {'trigger':CHECK_FEEDBACK_TRIGGER    , 'source':IDLE_STATE          , 'dest':CHECK_FEEDBACK_STATE, 'after': 'check_load_cell_feedback'},
            {'trigger':START_APPROACH_TRIGGER    , 'source':CHECK_FEEDBACK_STATE, 'dest':APPROACH_STATE      , 'after': 'finding_surface'         },
            {'trigger':SURFACE_FOUND_TRIGGER     , 'source':APPROACH_STATE      , 'dest':FIND_HOLE_STATE     , 'after': 'finding_hole'            },
            {'trigger':HOLE_FOUND_TRIGGER        , 'source':FIND_HOLE_STATE     , 'dest':INSERTING_PEG_STATE , 'after': 'inserting_peg'           },
            {'trigger':ASSEMBLY_COMPLETED_TRIGGER, 'source':INSERTING_PEG_STATE , 'dest':COMPLETION_STATE    , 'after': 'completed_insertion'     },

            {'trigger':SAFETY_RETRACTION_TRIGGER , 'source':IDLE_STATE          , 'dest':SAFETY_RETRACT_STATE, 'after': 'safety_retraction'       },
            {'trigger':SAFETY_RETRACTION_TRIGGER , 'source':APPROACH_STATE      , 'dest':SAFETY_RETRACT_STATE, 'after': 'safety_retraction'       },
            {'trigger':SAFETY_RETRACTION_TRIGGER , 'source':FIND_HOLE_STATE     , 'dest':SAFETY_RETRACT_STATE, 'after': 'safety_retraction'       },
            {'trigger':SAFETY_RETRACTION_TRIGGER , 'source':INSERTING_PEG_STATE , 'dest':SAFETY_RETRACT_STATE, 'after': 'safety_retraction'       },
            {'trigger':SAFETY_RETRACTION_TRIGGER , 'source':COMPLETION_STATE    , 'dest':SAFETY_RETRACT_STATE, 'after': 'safety_retraction'       },


        ]
        Machine.__init__(self, states=states, transitions=transitions, initial=IDLE_STATE)

        self._wrench_pub = rospy.Publisher('/cartesian_compliance_controller/target_wrench', WrenchStamped, queue_size=10)
        self._pose_pub = rospy.Publisher('cartesian_compliance_controller/target_frame', PoseStamped , queue_size=2)
        self._target_pub = rospy.Publisher('target_hole_position', PoseStamped, queue_size=2, latch=True)
        rospy.Subscriber("/cartesian_compliance_controller/ft_sensor_wrench/", WrenchStamped, self._callback_update_wrench, queue_size=2)
        
        #Needed to get current pose of the robot
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self._rate_selected = 100
        self._rate = rospy.Rate(self._rate_selected) #setup for sleeping in hz
        self._seq = 0
        self._start_time = rospy.get_rostime() #for _spiral_search_basic_force_control and _spiral_search_basic_compliance_control
        
        #TODO:Fix AssemblyStateEmitter(). Uncomment line below to get the error about hole_tol_plus not defined.
        #Create assembly state emitter for use with assembly algorithm's state machine
        # self._assembly_state = aes.AssemblyStateEmitter(self._rate_selected, self._start_time)

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
        rospy.logwarn_once('HERE IS THE POSE BELOW::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::')
        print(self.current_pose)
        self.current_wrench = self._first_wrench
        self._average_wrench = self._first_wrench.wrench 
        self._bias_wrench = self._first_wrench.wrench #Calculated to remove the steady-state error from wrench readings. 
        #TODO - subtract bias_wrench from the "current wrench" callback; Tried it but performance was unstable.
        self.average_speed = np.array([0.0,0.0,0.0])

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
        self.curr_time = rospy.get_rostime() - self._start_time
        self.curr_time_numpy = np.double(self.curr_time.to_sec())
        curr_amp = self._amp_c + self.safe_clearance * np.mod(2.0 * np.pi * self._freq_c *self.curr_time_numpy, self._amp_limit_c);

        # x_pos_offset = 0.88 #TODO:Assume the part needs to be inserted here at the offset. Fix with real value later
        # y_pos_offset = 0.550 #TODO:Assume the part needs to be inserted here at the offset. Fix with real value later
        
        # self._amp_c = self._amp_c * (self.curr_time_numpy * 0.001 * self.curr_time_numpy+ 1)

        x_pos = curr_amp * np.cos(2.0 * np.pi * self._freq_c *self.curr_time_numpy)
        x_pos = x_pos + self.x_pos_offset

        y_pos = curr_amp * np.sin(2.0 * np.pi * self._freq_c *self.curr_time_numpy)
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
    def _full_compliance_position(self, direction_vector = [0,0,0], desired_orientation = [0, 1, 0, 0]):
        #makes a command to simply stay still in a certain orientation
        pose_position = self.current_pose.transform.translation
        pose_position.x = pose_position.x + direction_vector[0]
        pose_position.y = pose_position.y + direction_vector[1]
        pose_position.z = pose_position.z + direction_vector[2]
        pose_orientation = desired_orientation
        return [[pose_position.x, pose_position.y, pose_position.z], pose_orientation]
    def _callback_update_wrench(self, data):
        self.current_wrench = data
        #self.current_wrench = data
        #self.current_wrench.wrench.force = self._subtract_vector3s(self.current_wrench.wrench.force, self._bias_wrench.force)
        #self.current_wrench.wrench.torque = self._subtract_vector3s(self.current_wrench.wrench.force, self._bias_wrench.force)
        #self.current_wrench.force = self._create_wrench([newForce[0], newForce[1], newForce[2]], [newTorque[0], newTorque[1], newTorque[2]]).wrench
        rospy.logwarn_once("Callback working! " + str(data))
    
    def _subtract_vector3s(self, vec1, vec2):
        newVector3 = Vector3(vec1.x - vec2.x, vec1.y - vec2.y, vec1.z - vec2.z)
        return newVector3

    def _get_current_pos(self):
        transform = self.tf_buffer.lookup_transform("base_link", "tool0", rospy.Time(0), rospy.Duration(100.0))
        return transform

    def _get_command_wrench(self, vec = [0,0,5]):
        self.curr_time = rospy.get_rostime() - self._start_time
        self.curr_time_numpy = np.double(self.curr_time.to_sec())

        # x_f = self._amp * np.cos(2.0 * np.pi * self._freq *self.curr_time_numpy)
        # y_f = self._amp * np.sin(2.0 * np.pi * self._freq *self.curr_time_numpy)
        x_f = vec[0]
        y_f = vec[1]
        z_f = vec[2] #apply constant downward force

        return [x_f, y_f, z_f, 0, 0, 0]

    def _calibrate_force_zero(self):
        self.curr_time = rospy.get_rostime() - self._start_time
        self.curr_time_numpy = np.double(self.curr_time.to_sec())

    def _publish_wrench(self, input_vec):
        # self.check_controller(self.force_controller)
        # forces, torques = self.com_to_tcp(result[:3], result[3:], transform)
        # result_wrench = self._create_wrench(result[:3], result[3:])
        # result_wrench = self._create_wrench([7,0,0], [0,0,0])
        result_wrench = self._create_wrench(input_vec[:3], input_vec[3:])
        
        self._wrench_pub.publish(result_wrench)

    # def _publish_pose(self, position, orientation):
    def _publish_pose(self, pose_stamped_vec):
        #Takes in vector representations of position vector (x,y,z) and orientation quaternion
        # Ensure controller is loaded
        # self.check_controller(self.controller_name)

        # Create poseStamped msg
        pose_stamped = PoseStamped()

        # Set the position and orientation
        point = Point()
        quaternion = Quaternion()

        # point.x, point.y, point.z = position
        point.x, point.y, point.z = pose_stamped_vec[0][:]
        pose_stamped.pose.position = point

        quaternion.w, quaternion.x, quaternion.y, quaternion.z  = pose_stamped_vec[1][:]
        pose_stamped.pose.orientation = quaternion

        # Set header values
        pose_stamped.header.stamp = rospy.get_rostime()
        pose_stamped.header.frame_id = "base_link"

        self._pose_pub.publish(pose_stamped)


        # r = rospy.Rate(30)

        # while 

        # # Publish pose to controller, ensuring with a lock only one callback publishes at a time
        # with self._pose_lock:
        #     # Get transform from the base_link to tool0
        #     # TODO: Documentation states args should be reversed, need to figure out why
        #     (trans,rot) = self.tf_listener.lookupTransform('/base_link', '/tool0', rospy.Time(0))
        #     rot = [rot[3], rot[0], rot[1], rot[2]] # Change quaternion order from xyzw to wxyz
        #     rot_matrix = tf.transformations.quaternion_matrix(rot)
        #     orientation_matrix = tf.transformations.quaternion_matrix(orientation)

        #     # Continue publishing until given destination is reached
        #     # TODO: Need to figure out a good way to check if the position is eventually reached, maybe a timeout?
        #     while not (np.allclose(trans, position, rtol=.02, atol=.02) and np.allclose(rot_matrix, orientation_matrix, rtol=.01, atol=.01)):
        #         self._pose_pub.publish(pose_stamped)
        #         # Update transform to current location
        #         (trans,rot) = self.tf_listener.lookupTransform('/base_link', '/tool0', rospy.Time(0))
        #         rot = [rot[3], rot[0], rot[1], rot[2]] # Change quaternion order from xyzw to wxyz
        #         rot_matrix = tf.transformations.quaternion_matrix(rot)
        #         r.sleep()

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

    def _update_average_wrench(self):
        #get a very simple average of wrench reading
        #self._average_wrench = self._weighted_average_wrenches(self._average_wrench, 9, self.current_wrench.wrench, 1)
        self._average_wrench = self._weighted_average_wrenches(self._average_wrench, 9, self.current_wrench.wrench, 1)
        #rospy.logwarn_throttle(.5, "Updating wrench toward " + str(self.current_wrench.wrench.force))

    def _weighted_average_wrenches(self, wrench1, scale1, wrench2, scale2):
        newForce = (self._as_array(wrench1.force) * scale1 + self._as_array(wrench2.force) * scale2) * 1/(scale1 + scale2)
        newTorque = (self._as_array(wrench1.torque) * scale1 + self._as_array(wrench2.torque) * scale2) * 1/(scale1 + scale2)
        return self._create_wrench([newForce[0], newForce[1], newForce[2]], [newTorque[0], newTorque[1], newTorque[2]]).wrench
            
    def _update_avg_speed(self):
        self.curr_time = rospy.get_rostime() - self._start_time
        if(self.curr_time.to_sec() > rospy.Duration(.5).to_sec()):
            #rospy.logwarn("Averageing speed! Time:" + str(self.curr_time.to_sec()))
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
            speedDiff = self._as_array(self.current_pose.transform.translation) - self._as_array(earlierPosition.transform.translation)
            timeDiff = ((self.current_pose.header.stamp) - (earlierPosition.header.stamp)).to_sec()
            if(timeDiff > 0.0): #Update only if we're using a new pose; also, avoid divide by zero
                speedDiff = speedDiff / timeDiff
                #Moving averate weighted toward old speed; response is now independent of rate selected.
                self.average_speed = self.average_speed * (1-10/self._rate_selected) + speedDiff * (10/self._rate_selected)
            #rospy.logwarn("Speed average: " + str(self.average_speed) )
        else:
            rospy.logwarn_throttle(1.0, "Too early to report past time!" + str(self.curr_time.to_sec()))
            
    @staticmethod
    def _as_array(vec):
        return np.array([vec.x, vec.y, vec.z])
    
    def _vectorRegionCompare_symmetrical(self, input, bounds_max):
        #initialize a minimum list
        bounds_min = [0,0,0] 
        #Each min value is the negative of the max value
        bounds_min[0] = bounds_max[0] * -1.0
        bounds_min[1] = bounds_max[1] * -1.0
        bounds_min[2] = bounds_max[2] * -1.0
        return self._vectorRegionCompare(input, bounds_max, bounds_min)
    
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

    def _force_cap_check(self):
        #Checks if any forces or torques are dangerously high.

        if(not (self._vectorRegionCompare_symmetrical(self._as_array(self.current_wrench.wrench.force), [45, 45, 45])
            and self._vectorRegionCompare_symmetrical(self._as_array(self.current_wrench.wrench.torque), [3.5, 3.5, 3.5]))):
                rospy.logerr("*Very* high force/torque detected! " + str(self.current_wrench.wrench))
                rospy.logerr("Killing program.")
                quit()
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
            rospy.sleep(1)
        return True
        
    def check_load_cell_feedback(self):
        switch_state = False
        #Take an average of static sensor reading to check that it's stable.
        while switch_state == False:

            self.all_states_calc()

            rospy.logwarn_throttle(0.5, 'In the check_load_cell_feedback. switch_state is:' + str(switch_state) )

            if (self.curr_time_numpy > 2):
                self._bias_wrench = self._average_wrench
                rospy.logerr("Measured bias wrench: " + str(self._bias_wrench))

                if( self._vectorRegionCompare_symmetrical(self._as_array(self._bias_wrench.torque), [1,1,1]) 
                and self._vectorRegionCompare_symmetrical(self._as_array(self._bias_wrench.force), [1.5,1.5,5])):
                    rospy.logerr("Starting linear search.")
                    switch_state = True
                    self.next_trigger = START_APPROACH_TRIGGER
                else:
                    rospy.logerr("Starting wrench is dangerously high. Suspending. Try restarting robot if values seem wrong.")
                    switch_state = True
                    self.next_trigger = SAFETY_RETRACTION_TRIGGER

            self._publish_pose(self.pose_vec)
            self._publish_wrench(self.wrench_vec)
            self._rate.sleep()

    def finding_surface(self):
        #seek in Z direction until we stop moving for about 1 second. 
        # Also requires "seeking_force" to be compensated pretty exactly by a static surface.
        #Take an average of static sensor reading to check that it's stable.
        switch_state = False
        while switch_state == False:

            self.all_states_calc()

            seeking_force = 5
            self.wrench_vec  = self._get_command_wrench([0,0,seeking_force])
            self.pose_vec = self._linear_search_position([0,0,0]) #doesn't orbit, just drops straight downward

            rospy.logwarn_throttle(2, 'In the finding_surface. switch_state is:' + str(switch_state))
 
            if(not self._force_cap_check()):
                switch_state = True
                self.next_trigger = SAFETY_RETRACTION_TRIGGER
                rospy.logerr("Force/torque unsafe; pausing application.")
            elif( self._vectorRegionCompare_symmetrical(self.average_speed, [5/1000,5/1000, 1/1000]) 
                and self._vectorRegionCompare(self._as_array(self.current_wrench.wrench.force), [2.5,2.5,seeking_force*-.75], [-2.5,-2.5,seeking_force*-1.25])):
                self.collision_confidence = self.collision_confidence + 1/self._rate_selected
                rospy.logerr_throttle(.5, "Monitoring for flat surface, confidence = " + str(self.collision_confidence))
                #if((rospy.Time.now()-marked_time).to_sec() > .50): #if we've satisfied this condition for 1 second
                if(self.collision_confidence > .90):
                    #Stopped moving vertically and in contact with something that counters push force
                    rospy.logerr("Flat surface detected! Moving to spiral search!")
                    #Measure flat surface height:
                    self.surface_height = self.current_pose.transform.translation.z
                    switch_state = True
                    self.next_trigger = SURFACE_FOUND_TRIGGER
                    self.collision_confidence = 0.01
 
            self._publish_pose(self.pose_vec)
            self._publish_wrench(self.wrench_vec)
            self._rate.sleep()

    def finding_hole(self):
        #Spiral until we descend 1/3 the specified hole depth (provisional fraction)
        #This triggers the hole position estimate to be updated to limit crazy
        #forces and oscillations. Also reduces spiral size.
        switch_state = False
        while switch_state == False:

            self.all_states_calc()

            seeking_force = 6.0
            self.wrench_vec  = self._get_command_wrench([0,0,seeking_force])
            self.pose_vec = self._spiral_search_basic_compliance_control()
 
            if(not self._force_cap_check()):
                switch_state = True
                self.next_trigger = SAFETY_RETRACTION_TRIGGER
                rospy.logerr("Force/torque unsafe; pausing application.")
            elif( self.current_pose.transform.translation.z <= self.surface_height - .0005):
                #If we've descended at least 5mm below the flat surface detected, consider it a hole.
                self.collision_confidence = self.collision_confidence + 1/self._rate_selected
                rospy.logerr_throttle(.5, "Monitoring for hole location, confidence = " + str(self.collision_confidence))
                if(self.collision_confidence > .90):
                        #Descended from surface detection point. Updating hole location estimate.
                        self.x_pos_offset = self.current_pose.transform.translation.x
                        self.y_pos_offset = self.current_pose.transform.translation.y
                        self._amp_limit_cp = 2 * np.pi * 4 #limits to 3 spirals outward before returning to center.
                        #TODO - Make these runtime changes pass as parameters to the "spiral_search_basic_compliance_control" function
                        rospy.logerr_throttle(1.0, "Hole found, peg inserting...")
                        switch_state = True
                        self.next_trigger = HOLE_FOUND_TRIGGER
            else:
                self.collision_confidence = np.max( np.array([self.collision_confidence * 95/self._rate_selected, .01]))
                if(self.current_pose.transform.translation.z >= self.surface_height - self.hole_depth):
                    rospy.logwarn_throttle(.5, "Height is still " + str(self.current_pose.transform.translation.z) 
                        + " whereas we should drop down to " + str(self.surface_height - self.hole_depth) )

            self._publish_pose(self.pose_vec)
            self._publish_wrench(self.wrench_vec)
            self._rate.sleep()

    
    def inserting_peg(self):
        #Continue spiraling downward. Outward normal force is used to verify that the peg can't move
        #horizontally. We keep going until vertical speed is very near to zero.
        switch_state = False
        while switch_state == False:

            self.all_states_calc()

            seeking_force = 5.0
            self.wrench_vec  = self._get_command_wrench([0,0,seeking_force])
            self.pose_vec = self._full_compliance_position()
 
            if(not self._force_cap_check()):
                switch_state = True
                self.next_trigger = SAFETY_RETRACTION_TRIGGER
                rospy.logerr("Force/torque unsafe; pausing application.")
            elif( self._vectorRegionCompare_symmetrical(self.average_speed, [2.5/1000,2.5/1000,.5/1000]) 
                #and not self._vectorRegionCompare(self._as_array(self.current_wrench.wrench.force), [6,6,80], [-6,-6,-80])
                and self._vectorRegionCompare(self._as_array(self.current_wrench.wrench.force), [1.5,1.5,seeking_force*-.75], [-1.5,-1.5,seeking_force*-1.25])
                and self.current_pose.transform.translation.z <= self.surface_height - self.hole_depth):
                self.collision_confidence = self.collision_confidence + 1/self._rate_selected
                rospy.logerr_throttle(.5, "Monitoring for peg insertion, confidence = " + str(self.collision_confidence))
                #if((rospy.Time.now()-marked_time).to_sec() > .50): #if we've satisfied this condition for 1 second
                if(self.collision_confidence > .90):
                        #Stopped moving vertically and in contact with something that counters push force
                        rospy.logerr_throttle(1.0, "Hole found, peg inserted! Done!")
                        switch_state = True
                        self.next_trigger = ASSEMBLY_COMPLETED_TRIGGER
            else:
                #rospy.logwarn_throttle(.5, "NOT a flat surface. Time: " + str((rospy.Time.now()-marked_time).to_sec()))
                self.collision_confidence = np.max( np.array([self.collision_confidence * 95/self._rate_selected, .01]))
                if(self.current_pose.transform.translation.z >= self.surface_height - self.hole_depth):
                    rospy.logwarn_throttle(.5, "Height is still " + str(self.current_pose.transform.translation.z) 
                        + " whereas we should drop down to " + str(self.surface_height - self.hole_depth) )
    
            self._publish_pose(self.pose_vec)
            self._publish_wrench(self.wrench_vec)
            self._rate.sleep()

    def completed_insertion(self):
        #Inserted properly.
        switch_state = False
        while switch_state == False:

            self.all_states_calc()

            rospy.logwarn_throttle(.50, "Hole found, peg inserted! Done!")
            if(self.current_pose.transform.translation.z > self.restart_height+.07):
                #High enough, won't pull itself upward.
                seeking_force = -2.5
            else:
                #pull upward gently to move out of trouble hopefully.
                seeking_force = -10
            self._force_cap_check()
            self.pose_vec = self._full_compliance_position()

            self._publish_pose(self.pose_vec)
            self._publish_wrench(self.wrench_vec)
            self._rate.sleep()

    def safety_retraction(self):
        #Safety passivation; chill and pull out. Actually restarts itself if everything's chill enough.

        switch_state = False
        while switch_state == False:

            self.all_states_calc()

            if(self.current_pose.transform.translation.z > self.restart_height+.05):
                #High enough, won't pull itself upward.
                seeking_force = -3.5
            else:
                #pull upward gently to move out of trouble hopefully.
                seeking_force = -7
            self.wrench_vec  = self._get_command_wrench([0,0,seeking_force])
            self.pose_vec = self._full_compliance_position()

            rospy.logerr_throttle(.5, "Task suspended for safety. Freewheeling until low forces and height reset above .20: " + str(self.current_pose.transform.translation.z))
            if( self._vectorRegionCompare_symmetrical(self.average_speed, [2/1000,2/1000,3/1000]) 
                and self._vectorRegionCompare_symmetrical(self._as_array(self.current_wrench.wrench.force), [1.5,1.5,5.5])
                and self.current_pose.transform.translation.z > self.restart_height):
                self.collision_confidence = self.collision_confidence + .5/self._rate_selected
                rospy.logerr_throttle(.5, "Static. Restarting confidence: " + str( np.round(self.collision_confidence, 2) ) + " out of 1.")
                #if((rospy.Time.now()-marked_time).to_sec() > .50): #if we've satisfied this condition for 1 second
                if(self.collision_confidence > 1):
                        #Stopped moving vertically and in contact with something that counters push force
                        rospy.logerr_throttle(1.0, "Restarting test!")
                        switch_state = True
                        self.next_trigger = ASSEMBLY_COMPLETED_TRIGGER
            else:
                self.collision_confidence = np.max( np.array([self.collision_confidence * 90/self._rate_selected, .01]))
                if(self.current_pose.transform.translation.z > self.restart_height):
                    rospy.logwarn_throttle(.5, "That's high enough! Let robot stop and come to zero force.")

            self._publish_pose(self.pose_vec)
            self._publish_wrench(self.wrench_vec)
            self._rate.sleep()

    #All state callbacks need to calculate this in a while loop
    def all_states_calc(self):
        #All once-per-loop functions
        self.current_pose = self._get_current_pos()
        self.curr_time = rospy.get_rostime() - self._start_time
        self.curr_time_numpy = np.double(self.curr_time.to_sec())
        marked_state = 1; #returns to this state after a soft restart in state 99
        self.wrench_vec  = self._get_command_wrench([0,0,-2])
        self.pose_vec = self._full_compliance_position()
        self._update_avg_speed()
        self._update_average_wrench()
        # self._update_plots()
        rospy.logwarn_throttle(.5, "Average wrench in newtons  is " + str(self._as_array(self._average_wrench.force))+ 
            str(self._as_array(self._average_wrench.torque)))
        rospy.logwarn_throttle(.5, "Average speed in mm/second is " + str(1000*self.average_speed))

       

    def _algorithm_compliance_control(self):
        
        # state = 0
        # cycle = 0
        self._average_wrench = self._first_wrench.wrench
        self.collision_confidence = 0
        
        rospy.logwarn_once('BELOW IS THE STATE BEFORE CHECK_FEEDBACK_TRIGGER')
        print(self.state)

        if not rospy.is_shutdown():
            self.trigger(CHECK_FEEDBACK_TRIGGER)

        while not rospy.is_shutdown():
            rospy.logwarn('BELOW IS THE STATE BEING TRANSITIONED FROM:')
            print(self.state)
            self.trigger(self.next_trigger)
            rospy.logwarn('BELOW IS THE STATE BEING TRANSITIONED TO:')
            print(self.state)        
            # self._publish_pose(self.pose_vec)
            # self._publish_wrench(self.wrench_vec)
            # self._rate.sleep()

    def main(self):
        # rospy.init_node("demo_assembly_application_compliance")

        # assembly_application = PegInHoleNodeCompliance()
        # assembly_application._algorithm_force_control()

        #---------------------------------------------COMPLIANCE CONTROL BELOW, FORCE CONTROL ABOVE
        rospy.logwarn_once('MADE IT TO MAIN FUNCTION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        rospy.sleep(3.5)
        # assembly_application._init_plot()

        self._algorithm_compliance_control()

if __name__ == '__main__':
    
    assembly_application = PegInHoleNodeCompliance()

    assembly_application.main()
    
    



# from gripper import Gripper

# def __init__(self, config):
#         self.config = config # :type dict:
#         self.actions = []
#         self._action_map = {
#             'pause':        self.create_pause_action,
#             'movej':        self.create_joint_move_action,
#             'movec':        self.create_cartesian_move_action,
#             'brake':        self.create_brake_action,
#             'gripper':      self.create_gripper_action,
#         }
        
#         self._gripper_action_map = {
#             'activate':     ActivateGripper,
#             'reset':        ResetGripper,
#             'open':         OpenGripper,
#             'close':        CloseGripper,
#         }


# def create_gripper_action(self, step):
#         state = step[Plan._STATE_KEY]
#         if state in self._gripper_action_map:
#             self.actions.append(self._gripper_action_map[state](step[
#                 Plan._MESSAGE_KEY]))
#         else:
#             raise Exception('Unknown gripper action: %s' % state)


# #The gripper doesn't need to be reset each time a new script communicates with it, but it does need to be activated.
# #But it doesn't hurt to reset it before activating each time, because sometimes it does need to be reset

# class ResetGripper(Action):
#     def __init__(self, message):
#         self.message = message

#     def execute(self, control):
#         control.iiwa_robot.gripper.reset()

#     def __str__(self):
#         return "ResetGripper"

# class ActivateGripper(Action):
#     def __init__(self, message):
#         self.message = message

#     def execute(self, control):
#         control.iiwa_robot.gripper.activate()

#     def __str__(self):
#         return "ActivateGripper"

# class OpenGripper(Action):
#     def __init__(self, message):
#         self.message = message

#     def execute(self, control):
#         control.iiwa_robot.gripper.open(self.message)

#     def __str__(self):
#         return "OpenGripper"

# class CloseGripper(Action):
#     def __init__(self, message):
#         self.message = message

#     def execute(self, control):
#         control.iiwa_robot.gripper.close(self.message)

#     def __str__(self):
#         return "CloseGripper"

# class Pause(Action):
#     def __init__(self, message):
#         self.message = message

#     def execute(self, control):
#         input('{} - press enter to continue...'.format(self.message))

#     def __str__(self):
#         return "Pause"



