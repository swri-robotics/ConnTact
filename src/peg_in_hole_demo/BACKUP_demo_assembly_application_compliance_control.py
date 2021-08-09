#!/usr/bin/env python

# Imports for ros
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
import tf2_py
import tf2_ros
from tf.transformations import quaternion_from_euler
# import tf2
import tf2_geometry_msgs

from threading import Lock

#import custom state function
import assembly_state_emitter as aes

class PegInHoleNodeCompliance():

    def __init__(self):

        self._assembly_state = AssemblyStateEmitter()

        self._wrench_pub = rospy.Publisher('/cartesian_compliance_controller/target_wrench', WrenchStamped, queue_size=10)
        self._pose_pub = rospy.Publisher('cartesian_compliance_controller/target_frame', PoseStamped , queue_size=2)
        self._target_pub = rospy.Publisher('target_hole_position', PoseStamped, queue_size=2, latch=True)
        rospy.Subscriber("/cartesian_compliance_controller/ft_sensor_wrench/", WrenchStamped, self._callback_update_wrench, queue_size=2)
        
        #Needed to get current pose of the robot
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.rateSelected = 100
        self.rate = rospy.Rate(self.rateSelected) #setup for sleeping in hz
        self._seq = 0
        self._start_time = rospy.get_rostime() #for _spiral_search_basic_force_control and _spiral_search_basic_compliance_control
        
        #Spiral parameters
        self._freq = np.double(0.15) #Hz frequency in _spiral_search_basic_force_control
        self._amp  = np.double(10.0)  #Newton amplitude in _spiral_search_basic_force_control
        self._first_wrench = self._create_wrench([0,0,0], [0,0,0])
        self._freq_c = np.double(0.15) #Hz frequency in _spiral_search_basic_compliance_control
        self._amp_c  = np.double(.002)  #meters amplitude in _spiral_search_basic_compliance_control
        self._amp_limit_c = 2 * np.pi * 10 #search number of radii distance outward

        #job parameters, should be moved in from the peg_in_hole_params.yaml file
        
        # rospy.logerr("Hole depth is: " + str(self.hole_depth))
        temp_z_position_offset = 207/1000 #Our robot is reading Z positions wrong on the pendant for some reason.
        taskPos = list(np.array(rospy.get_param('/environment_state/task_frame/position'))/1000)
        taskPos[2] = taskPos[2] + temp_z_position_offset
        taskOri = rospy.get_param('/environment_state/task_frame/orientation')
        holePos = list(np.array(rospy.get_param('/objects/hole/local_position'))/1000)
        holePos[2] = holePos[2] + temp_z_position_offset
        holeOri = rospy.get_param('/objects/hole/local_orientation')
        
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

        rospy.logerr("Hole Pose: " + str(self.target_hole_pose))
        self._target_pub.publish(self.target_hole_pose)
        #self.x_pos_offset = 0.532
        #self.y_pos_offset = -0.171 
        self.x_pos_offset = self.target_hole_pose.pose.position.x
        self.y_pos_offset = self.target_hole_pose.pose.position.y

        peg_diameter = rospy.get_param('/objects/peg/dimensions/diameter')/1000 #mm
        peg_tol_plus = rospy.get_param('/objects/peg/tolerance/upper_tolerance')/1000
        peg_tol_minus = rospy.get_param('/objects/peg/tolerance/lower_tolerance')/1000

        hole_diameter = rospy.get_param('/objects/hole/dimensions/diameter')/1000 #mm
        hole_tol_plus = rospy.get_param('/objects/hole/tolerance/upper_tolerance')/1000
        hole_tol_minus = rospy.get_param('/objects/hole/tolerance/lower_tolerance')/1000    
        self.hole_depth = rospy.get_param('/objects/peg/dimensions/min_insertion_depth')/1000
        #self.hole_depth = rospy.get_param('/objects/peg/dimensions/length', )
        #self.hole_depth = .0075 #we need to insert at least this far before it will consider if it's inserted
        
        #loop parameters
        self.current_pose = self._get_current_pos()
        self.current_wrench = self._first_wrench
        self._average_wrench = self._first_wrench.wrench 
        self._bias_wrench = self._first_wrench.wrench #Calculated to remove the steady-state error from wrench readings. 
        #TODO - subtract bias_wrench from the "current wrench" callback; Tried it but performance was unstable.
        self.average_speed = np.array([0.0,0.0,0.0])

        #Simple Moving Average Parameters
        self._buffer_window = self.rateSelected #1/Hz since this variable is the rate of ROS commands
        self._moving_avg_data = np. #Empty to start. make larger than we need since np is contiguous memory. Will ignore NaN values.
        # self._moving_avg_data = [] #Empty to start

        #plotting parameters
        self.speedHistory = np.array(self.average_speed)
        self.forceHistory = self._as_array(self._average_wrench.force)
        self.posHistory = np.array([self.x_pos_offset*1000, self.y_pos_offset*1000, self.current_pose.transform.translation.z*1000])
        self.plotTimes = [0]
        self.recordInterval = rospy.Duration(.2)
        self.plotInterval = rospy.Duration(.5)
        self.lastPlotted = rospy.Time(0)
        self.lastRecorded = rospy.Time(0)
        self.recordLength = 50
        self.surface_height = None
        self.restart_height = .1
        self.highForceWarning = False

        #setup, run to calculate useful values based on params:
        self.clearance_max = hole_tol_plus - peg_tol_minus #calculate the total error zone;
        self.clearance_min = hole_tol_minus + peg_tol_plus #calculate minimum clearance;     =0
        self.clearance_avg = .5 * (self.clearance_max- self.clearance_min) #provisional calculation of "wiggle room"
        self.safe_clearance = (hole_diameter-peg_diameter + self.clearance_min)/2; # = .2 *radial* clearance i.e. on each side.

        
        

    def _spiral_search_basic_compliance_control(self):
        _assembl
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
    
    #Should move to a new node on a different thread that does not bog down controller. 
    def _init_plot(self):
            #plt.axis([-50,50,0,10000])
        plt.ion()
        # plt.show()
        # plt.draw()

        self.fig, (self.planView, self.sideView, self.forceDisplay) = plt.subplots(1, 3)
        self.fig.suptitle('Horizontally stacked subplots')
        plt.subplots_adjust(left=.1, bottom=.2, right=.975, top=.8, wspace=.15, hspace=.1)

        self.min_plot_window_offset = 3
        self.min_plot_window_size = 15
        self.pointOffset = 1
        self.barb_interval = 5

    #Should move to a new node on a different thread that does not bog down controller. 
    def _update_plots(self):
        if(rospy.Time.now() > self.lastRecorded + self.recordInterval):
            self.lastRecorded = rospy.Time.now()

            #log all interesting data
            self.speedHistory = np.vstack((self.speedHistory, np.array(self.average_speed)*1000))
            self.forceHistory = np.vstack((self.forceHistory, self._as_array(self._average_wrench.force)))
            self.posHistory = np.vstack((self.posHistory, self._as_array(self.current_pose.transform.translation)*1000))
            self.plotTimes.append((rospy.get_rostime() - self._start_time).to_sec())
            
            #limit list lengths
            if(len(self.speedHistory)>self.recordLength):
                self.speedHistory = self.speedHistory[1:-1]
                self.forceHistory = self.forceHistory[1:-1]
                self.posHistory = self.posHistory[1:-1]
                self.plotTimes = self.plotTimes[1:-1]
                self.pointOffset += 1

        if(rospy.Time.now() > self.lastPlotted + self.plotInterval):
            
            self.lastPlotted = rospy.Time.now()

            self.planView.clear()
            self.sideView.clear()
            self.forceDisplay.clear()

            self.planView.set(xlabel='X Position',ylabel='Y Position')
            self.planView.set_title('Position and Force')
            self.sideView.set(xlabel='Time (s)',ylabel='Position (mm) and Force (N)')
            self.sideView.set_title('Vertical position and force')
            self.forceDisplay.set(xlabel=' ',ylabel=' ')
            self.forceDisplay.set_title('Forces and Velocities')

            self.planView.plot(self.posHistory[:,0], self.posHistory[:,1], 'r')
            #self.planView.quiver(self.posHistory[0:-1:10,0], self.posHistory[0:-1:10,1], self.forceHistory[0:-1:10,0], self.forceHistory[0:-1:10,1], angles='xy', scale_units='xy', scale=.1, color='b')
            barb_increments = {"flag" :5, "full" : 1, "half" : 0.5}
            barb_increments_2 = {"flag" :.005, "full" : .001, "half" : 0.0005}

            offset = self.barb_interval+1-(self.pointOffset % self.barb_interval)
            self.planView.barbs(self.posHistory[offset:-1:self.barb_interval,0], self.posHistory[offset:-1:self.barb_interval,1], 
                self.forceHistory[offset:-1:self.barb_interval,0], self.forceHistory[offset:-1:self.barb_interval,1]*-1,
                barb_increments=barb_increments, length = 6, color=(0.3, 0.7, 0.7))
            #TODO - Fix barb directions. I think they're pointing the wrong way, just watching them in freedrive mode.

            self.sideView.plot(self.plotTimes, self.forceHistory[:,2], 'k', self.plotTimes, self.posHistory[:,2], 'b')

            xlims = [np.min(self.posHistory[:,0]) - self.min_plot_window_offset, np.max(self.posHistory[:,0]) + self.min_plot_window_offset]
            if xlims[1] - xlims[0] < self.min_plot_window_size:
                xlims[0] -= self.min_plot_window_size / 2
                xlims[1] += self.min_plot_window_size / 2

            ylims = [np.min(self.posHistory[:,1]) - self.min_plot_window_offset, np.max(self.posHistory[:,1]) + self.min_plot_window_offset]
            if ylims[1] - ylims[0] < self.min_plot_window_size:
                ylims[0] -= self.min_plot_window_size / 2
                ylims[1] += self.min_plot_window_size / 2
            
            self.planView.set_xlim(xlims)
            self.planView.set_ylim(ylims)

                
            self.forceDisplay.set_xlim(-2, 2)
            self.forceDisplay.set_ylim(-2, 6)

            #display current forces in 2 barbs
            self.forceDisplay.barbs(0,0,self.current_wrench.wrench.force.x,-1*self.current_wrench.wrench.force.y,
                barb_increments=barb_increments, length = 8, color=(0.3, 0.7, 0.7))
            self.forceDisplay.barbs(-.1,4,0,-1*self.current_wrench.wrench.force.z,
                barb_increments=barb_increments, length = 8, color=(0.3, 0.7, 0.7))
            #display current speed in 2 barbs
            self.forceDisplay.barbs(0,0,self.average_speed[0],self.average_speed[1],
                barb_increments=barb_increments_2, length = 8, color=(0.7, 0.0, 0.0))
            self.forceDisplay.barbs(0.1,4,0,self.average_speed[2],
                barb_increments=barb_increments_2, length = 8, color=(0.7, 0.0, 0.0))
                        
            if not self.surface_height is None:
                self.sideView.axhline(y=self.surface_height*1000, color='r', linestyle='-')

            self.fig.canvas.draw()
            # plt.pause(0.001)
            #plt.show()

            #x velocity
            #plt.scatter(self.speedHistory[:][0], )

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
    
    # Not sure if working correctly. 9 abnd 1 are weights. Does not work well when you change selected rate variable.
    #The faster you run the function, the faster it responds to changes in the force/torque. 
    #Trying to create a slow moving average of the force and torque data. Maybe get numpy/scipi low-pass implementation
    def _update_average_wrench(self):
        #get a very simple average of wrench reading
        #self._average_wrench = self._weighted_average_wrenches(self._average_wrench, 9, self.current_wrench.wrench, 1)
        self._average_wrench = self._weighted_average_wrenches(self._average_wrench, 9, self.current_wrench.wrench, 1)
        #rospy.logwarn_throttle(.5, "Updating wrench toward " + str(self.current_wrench.wrench.force))

    def _simple_moving_average(self, realtime_data, buffer_window=self._buffer_window)
        self._moving_avg_data = self._moving_avg_data.append(wrenchstamped.value)
        if length(self._moving_avg_data < buffer_window)
        return avg

        

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
                self.average_speed = self.average_speed * (1-10/self.rateSelected) + speedDiff * (10/self.rateSelected)
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
        
    def _algorithm_compliance_control(self):
        
        state = 0
        cycle = 0
        self._average_wrench = self._first_wrench.wrench #TODO: Why is sthis line here? Delete?
        collision_confidence = 0

        while (state < 100) and (not rospy.is_shutdown()):
            #TODO: put the below code in a function according to John.
            #All once-per-loop functions
            self.current_pose = self._get_current_pos()
            curr_time = rospy.get_rostime() - self._start_time
            curr_time_numpy = np.double(curr_time.to_sec())
            marked_state = 1; #returns to this state after a soft restart in state 99
            wrench_vec  = self._get_command_wrench([0,0,-2])
            pose_vec = self._full_compliance_position()
            self._update_avg_speed()
            self._update_average_wrench()
            self._update_plots()
            #rospy.logwarn_throttle(.5, "Average wrench in newtons  is " + str(self._as_array(self._average_wrench.force))+ 
            #    str(self._as_array(self._average_wrench.torque)))
            rospy.logwarn_throttle(.5, "Average speed in mm/second is " + str(1000*self.average_speed))
            

            if (state == 0): 
                #Take an average of static sensor reading to check that it's stable.                
                if (curr_time_numpy > 2):
                    self._bias_wrench = self._average_wrench
                    rospy.logerr("Measured bias wrench: " + str(self._bias_wrench))
                    
                    if( self._vectorRegionCompare_symmetrical(self._as_array(self._bias_wrench.torque), [1,1,1]) 
                    #The 5 Newton on Z is because the gripper weighs 5 N. We apply 5 N again later to send it downward (by coincidence)
                    and self._vectorRegionCompare_symmetrical(self._as_array(self._bias_wrench.force), [1.5,1.5,5])):
                        rospy.logerr("Starting linear search.")
                        state = 1
                    else:
                        rospy.logerr("Starting wrench is dangerously high. Suspending. Try restarting robot if values seem wrong.")
                        state = 99
            elif (state == 1): 
                #seek in Z direction until we stop moving for about 1 second. 
                # Also requires "seeking_force" to be compensated pretty exactly by a static surface.
                seeking_force = 5
                wrench_vec  = self._get_command_wrench([0,0,seeking_force])
                pose_vec = self._linear_search_position([0,0,0]) #doesn't orbit, just drops straight downward
                
                if(not self._force_cap_check()):
                    state = 99
                    rospy.logerr("Force/torque unsafe; pausing application.")
                    #Below line works. The z is small because the flat surface resists, x and y may drift. 
                elif( self._vectorRegionCompare_symmetrical(self.average_speed, [5/1000,5/1000, 1/1000]) #divide by 1000 to convert to mm/s
                    #Not doing a symmetrical boundary in the next line. 
                    and self._vectorRegionCompare(self._as_array(self.current_wrench.wrench.force), [2.5,2.5,seeking_force*-.75], [-2.5,-2.5,seeking_force*-1.25])):
                    collision_confidence = collision_confidence + 1/self.rateSelected
                    rospy.logerr_throttle(.5, "Monitoring for flat surface, confidence = " + str(collision_confidence))
                    #if((rospy.Time.now()-marked_time).to_sec() > .50): #if we've satisfied this condition for 1 second
                    if(collision_confidence > .90):
                            #Stopped moving vertically and in contact with something that counters push force
                            rospy.logerr("Flat surface detected! Moving to spiral search!")
                            #Measure flat surface height:
                            self.surface_height = self.current_pose.transform.translation.z
                            state = 2
                            collision_confidence = 0.01
                else:
                    #rospy.logwarn_throttle(.5, "NOT a flat surface. Time: " + str((rospy.Time.now()-marked_time).to_sec()))
                    #TODO: Fix math by maybe .95 * self.rateSelected? It should produce a concave up curve when not choosing 0.01.
                    collision_confidence = np.max( np.array([collision_confidence * 95/self.rateSelected, .01]))
                    #marked_time = rospy.Time.now()
            elif (state == 2):
                #Spiral until we descend 1/3 the specified hole depth (provisional fraction)
                #This triggers the hole position estimate to be updated to limit crazy
                #forces and oscillations. Also reduces spiral size.
                seeking_force = 6.0
                wrench_vec  = self._get_command_wrench([0,0,seeking_force])
                pose_vec = self._spiral_search_basic_compliance_control()
                
                if(not self._force_cap_check()):
                    state = 99
                    rospy.logerr("Force/torque unsafe; pausing application.")
                elif( self.current_pose.transform.translation.z <= self.surface_height - .0005):
                    #If we've descended at least 5mm below the flat surface detected, consider it a hole.
                    collision_confidence = collision_confidence + 1/self.rateSelected
                    rospy.logerr_throttle(.5, "Monitoring for hole location, confidence = " + str(collision_confidence))
                    if(collision_confidence > .90):
                            #Descended from surface detection point. Updating hole location estimate.
                            self.x_pos_offset = self.current_pose.transform.translation.x
                            self.y_pos_offset = self.current_pose.transform.translation.y
                            self._amp_limit_cp = 2 * np.pi * 4 #limits to 3 spirals outward before returning to center.
                            #TODO - Make these runtime changes pass as parameters to the "spiral_search_basic_compliance_control" function
                            rospy.logerr_throttle(1.0, "Hole found, peg inserting...")
                            state = 3
                else:
                    collision_confidence = np.max( np.array([collision_confidence * 95/self.rateSelected, .01]))
                    if(self.current_pose.transform.translation.z >= self.surface_height - self.hole_depth):
                        rospy.logwarn_throttle(.5, "Height is still " + str(self.current_pose.transform.translation.z) 
                            + " whereas we should drop down to " + str(self.surface_height - self.hole_depth) )
            elif (state == 3):
                #Continue  downward. We keep going until vertical speed is very near to zero.
                seeking_force = 5.0
                wrench_vec  = self._get_command_wrench([0,0,seeking_force])
                pose_vec = self._full_compliance_position()
                
                if(not self._force_cap_check()):
                    state = 99
                    rospy.logerr("Force/torque unsafe; pausing application.")
                elif( self._vectorRegionCompare_symmetrical(self.average_speed, [2.5/1000,2.5/1000,.5/1000]) 
                    #and not self._vectorRegionCompare(self._as_array(self.current_wrench.wrench.force), [6,6,80], [-6,-6,-80])
                    and self._vectorRegionCompare(self._as_array(self.current_wrench.wrench.force), [1.5,1.5,seeking_force*-.75], [-1.5,-1.5,seeking_force*-1.25])
                    and self.current_pose.transform.translation.z <= self.surface_height - self.hole_depth):
                    collision_confidence = collision_confidence + 1/self.rateSelected
                    rospy.logerr_throttle(.5, "Monitoring for peg insertion, confidence = " + str(collision_confidence))
                    #if((rospy.Time.now()-marked_time).to_sec() > .50): #if we've satisfied this condition for 1 second
                    if(collision_confidence > .90):
                            #Stopped moving vertically and in contact with something that counters push force
                            rospy.logerr_throttle(1.0, "Hole found, peg inserted! Done!")
                            state = 4
                else:
                    #rospy.logwarn_throttle(.5, "NOT a flat surface. Time: " + str((rospy.Time.now()-marked_time).to_sec()))
                    collision_confidence = np.max( np.array([collision_confidence * 95/self.rateSelected, .01]))
                    if(self.current_pose.transform.translation.z >= self.surface_height - self.hole_depth):
                        rospy.logwarn_throttle(.5, "Height is still " + str(self.current_pose.transform.translation.z) 
                            + " whereas we should drop down to " + str(self.surface_height - self.hole_depth) )
            elif (state == 4):
                #Inserted properly.
                rospy.logwarn_throttle(.50, "Hole found, peg inserted! Done!")
                if(self.current_pose.transform.translation.z > self.restart_height+.07):
                    #High enough, won't pull itself upward.
                    seeking_force = -2.5
                else:
                    #pull upward gently to move out of trouble hopefully.
                    seeking_force = -15
                self._force_cap_check()
                pose_vec = self._full_compliance_position()
                
            elif (state == 99):
                #Safety passivation; chill and pull out. Actually restarts itself if everything's chill enough.
                if(self.current_pose.transform.translation.z > self.restart_height+.05):
                    #High enough, won't pull itself upward.
                    seeking_force = -3.5
                else:
                    #pull upward gently to move out of trouble hopefully.
                    seeking_force = -10
                wrench_vec  = self._get_command_wrench([0,0,seeking_force])
                pose_vec = self._full_compliance_position()
                                                
                rospy.logerr_throttle(.5, "Task suspended for safety. Freewheeling until low forces and height reset above .20: " + str(self.current_pose.transform.translation.z))

                if( self._vectorRegionCompare_symmetrical(self.average_speed, [3/1000,3/1000,3/1000]) 
                    and self._vectorRegionCompare_symmetrical(self._as_array(self.current_wrench.wrench.force), [1.5,1.5,5.5])
                    and self.current_pose.transform.translation.z > self.restart_height):
                    collision_confidence = collision_confidence + .5/self.rateSelected
                    rospy.logerr_throttle(.5, "Static. Restarting confidence: " + str( np.round(collision_confidence, 2) ) + " out of 1.")
                    #if((rospy.Time.now()-marked_time).to_sec() > .50): #if we've satisfied this condition for 1 second
                    if(collision_confidence > 1):
                            #Stopped moving vertically and in contact with something that counters push force
                            rospy.logerr_throttle(1.0, "Restarting test!")
                            state = marked_state
                else:
                    collision_confidence = np.max( np.array([collision_confidence * 90/self.rateSelected, .01]))
                    if(self.current_pose.transform.translation.z > self.restart_height):
                        rospy.logwarn_throttle(.5, "That's high enough! Let robot stop and come to zero force.")

            self._publish_pose(pose_vec)
            self._publish_wrench(wrench_vec)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node("demo_assembly_application_compliance")
    
    # assembly_application = PegInHoleNodeCompliance()
    # assembly_application._algorithm_force_control()

    #---------------------------------------------COMPLIANCE CONTROL BELOW, FORCE CONTROL ABOVE
    rospy.sleep(3.5)
    assembly_application = PegInHoleNodeCompliance()
    assembly_application._init_plot()
    assembly_application._algorithm_compliance_control()



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


