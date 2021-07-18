#!/usr/bin/env python

# Imports for ros
from operator import truediv
import rospy
# import tf
import numpy as np
import matplotlib.pyplot as plt
from rospkg import RosPack
from geometry_msgs.msg import WrenchStamped, Wrench, TransformStamped, PoseStamped, Pose, Point, Quaternion, Vector3
from rospy.core import configure_logging

from sensor_msgs.msg import JointState
# from assembly_ros.srv import ExecuteStart, ExecuteRestart, ExecuteStop
from controller_manager_msgs.srv import SwitchController, LoadController, ListControllers

import tf2_ros
# import tf2
import tf2_geometry_msgs

from threading import Lock

class PegInHoleNodeCompliance():

    def __init__(self):
        self._wrench_pub = rospy.Publisher('/cartesian_compliance_controller/target_wrench', WrenchStamped, queue_size=10)
        self._pose_pub = rospy.Publisher('cartesian_compliance_controller/target_frame', PoseStamped , queue_size=2)
        rospy.Subscriber("/cartesian_compliance_controller/ft_sensor_wrench/", WrenchStamped, self._callback_update_wrench, queue_size=2)
        
        plt.show()
        #Needed to get current pose of the robot
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        # self.tf_buffer = tf2.BufferCore(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.rateSelected = 100
        self.rate = rospy.Rate(self.rateSelected) #setup for sleeping in hz
        self._seq = 0
        self._start_time = rospy.get_rostime() #for _spiral_search_basic_force_control and _spiral_search_basic_compliance_control
        self._freq = np.double(0.15) #Hz frequency in _spiral_search_basic_force_control
        self._amp  = np.double(10.0)  #Newton amplitude in _spiral_search_basic_force_control
        self._first_wrench = self._create_wrench([0,0,0], [0,0,0])
        


        self._freq_c = np.double(0.15) #Hz frequency in _spiral_search_basic_compliance_control
        self._amp_c  = np.double(.002)  #meters amplitude in _spiral_search_basic_compliance_control
        self._amp_limit_c = 2*np.pi * 7; #search number of radii distance outward

        #job parameters, should be moved to a yaml file by a wizard
        self.x_pos_offset = 0.539 #TODO:Assume the part needs to be inserted here at the offset. Fix with real value later
        self.y_pos_offset = -0.238 #TODO:Assume the part needs to be inserted here at the offset. Fix with real value later

        peg_diameter = 16/1000 #mm
        peg_tol_plus = 0.0/1000
        peg_tol_minus = -.01/1000

        hole_diameter = 16.4/1000 #mm
        hole_tol_plus = .01/1000
        hole_tol_minus = 0.0/1000
        self.hole_depth = .0075 #we need to insert at least this far before it will consider if it's inserted

        
        self.current_pose = self._get_current_pos
        self.current_wrench = self._first_wrench
        self._average_wrench = self._first_wrench.wrench 
        self._bias_wrench = self._first_wrench.wrench #Calculated to remove the steady-state error from wrench readings. 
        #TODO - subtract bias_wrench from the "current wrench" callback; Tried it but performance was unstable.
        self.average_speed = np.array([0.0,0.0,0.0])


        self.speedHistory = [self.average_speed]
        self.plotTimes = [0]
        self.plotInterval = 500;
        self.lastPlotted = rospy.Time(0).to_sec();

        #setup:
        self.clearance_max = hole_tol_plus - peg_tol_minus #calculate the total error zone;
        self.clearance_min = hole_tol_minus + peg_tol_plus #calculate minimum clearance;     =0
        self.clearance_avg = .5 * (self.clearance_max- self.clearance_min) #provisional calculation of "wiggle room"
        self.safe_clearance = (hole_diameter-peg_diameter + self.clearance_min)/2; # = .2 *radial* clearance i.e. on each side.

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
        # transform = self.tf_buffer.lookup_transform("tool0",
        # "base_link", #source frame
        # # pose_stamped_to_transform.header.base_link, #source frame
        # rospy.get_rostime(), #get the tf at the current time
        # # rospy.Time(0), #get the tf at first available time
        # rospy.Duration(1.0)) #wait for 1 second
        # # pose_transform = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        
        transform = self.tf_buffer.lookup_transform("base_link", "tool0", rospy.Time(0), rospy.Duration(100.0))
        # rospy.logerr("The type is ")
        # rospy.logerr(type(_transform))

        #return the z position only of the pose
        return transform

    def _get_command_wrench(self, vec = [0,0,5]):
        curr_time = rospy.get_rostime() - self._start_time
        curr_time_numpy = np.double(curr_time.to_sec())

        # x_f = self._amp * np.cos(2.0 * np.pi * self._freq *curr_time_numpy)
        # y_f = self._amp * np.sin(2.0 * np.pi * self._freq *curr_time_numpy)
        x_f = vec[0]
        y_f = vec[1]
        z_f = vec[2] #apply constant downward force

        return [x_f, y_f, z_f, 0, 0, 0]
    def _calibrate_force_zero(self):
        curr_time = rospy.get_rostime() - self._start_time
        curr_time_numpy = np.double(curr_time.to_sec())
    def _update_plots(self):
        if(rospy.Time.now.to_secs() > self.lastPlotted + self.plotInterval ):
            self.speedHistory.append(self.average_speed)
            self.plotTimes.append(rospy.get_rostime() - self._start_time)
            plt.cla()
            plt.xlabel('Time (s)')
            plt.ylabel('Velocity, m/s')
            plt.title('Sped')
            plt.plot(self.speedHistory[0,:], )

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

    def _ft_sensor_callback():
        # Update current data from force sensor
        rospy.logwarn("Sensor Callback triggered!")
        forces = sensor_wrench.wrench.force
        torques = sensor_wrench.wrench.torque
        # forces, torques = self.tcp_to_com(forces, torques)
        self._first_wrench = self._create_wrench([forces.x, forces.y, forces.z,
                        torques.x, torques.y, torques.z])

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
            speedDiff = self._as_array(self.current_pose.transform.translation) - self._as_array(earlierPosition.transform.translation)
            timeDiff = ((self.current_pose.header.stamp) - (earlierPosition.header.stamp)).to_sec()
            if(timeDiff > 0.0): #Update only if we're using a new pose; also, avoid divide by zero
                speedDiff = speedDiff / timeDiff
                #Moving averate weighted toward old speed; response is now independent of rate selected.
                self.average_speed = self.average_speed * (1-10/self.rateSelected) + speedDiff * (10/self.rateSelected)
            #rospy.logwarn("Speed average: " + str(self.average_speed) )
        else:
            rospy.logwarn_throttle(1.0, "Too early to report past time!" + str(curr_time.to_sec()))
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
        if( bounds_max[0] >= input[0] >= bounds_min[0]):
            if( bounds_max[1] >= input[1] >= bounds_min[1]):
                if( bounds_max[2] >= input[2] >= bounds_min[2]):
                    #rospy.logwarn(.5, "_______________ping!________________")
                    return True
        return False
    def _force_cap_check(self):
        if(self._vectorRegionCompare_symmetrical(self._as_array(self.current_wrench.wrench.force), [30, 30, 30])):
            if(self._vectorRegionCompare_symmetrical(self._as_array(self.current_wrench.wrench.torque), [2.2, 2.2, 2.2])):
                return True
        rospy.logerr("High force/torque detected! " + str(self.current_wrench.wrench))
        #return True
        return False

    def _algorithm_force_control(self):

        self.rate = rospy.Rate(50) #setup for sleeping at 10hz
        while not rospy.is_shutdown():           

            command_vec = self._spiral_search_basic_force_control()
            self._publish(command_vec)

            self.rate.sleep()

    def _algorithm_compliance_control(self):
        
        state = 0
        cycle = 0
        self._average_wrench = self._first_wrench.wrench
        collision_confidence = 0
        surface_height = .25

        while (state < 100) and (not rospy.is_shutdown()):
            self.current_pose = self._get_current_pos()
            curr_time = rospy.get_rostime() - self._start_time
            curr_time_numpy = np.double(curr_time.to_sec())
            marked_state = 1; #returns to this state after a soft restart in state 99
            wrench_vec  = self._get_command_wrench([0,0,-2])
            pose_vec = self._full_compliance_position()
            self._update_avg_speed()
            self._update_average_wrench()
            rospy.logwarn_throttle(.5, "Average wrench in newtons  is " + str(self._as_array(self._average_wrench.force))+ str(self._as_array(self._average_wrench.torque)))
            rospy.logwarn_throttle(.5, "Average speed in mm/second is " + str(1000*self.average_speed))
            

            if (state == 0): 
                #Take an average of static sensor reading to check that it's stable.                
                if (curr_time_numpy > 2):
                    self._bias_wrench = self._average_wrench
                    rospy.logerr("Measured bias wrench: " + str(self._bias_wrench))
                    
                    if( self._vectorRegionCompare_symmetrical(self._as_array(self._bias_wrench.torque), [1,1,1]) 
                    and self._vectorRegionCompare_symmetrical(self._as_array(self._bias_wrench.force), [1,1,5])):
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
                    rospy.logerr("Very high force/torque detected; going limp and ending test.")
                elif( self._vectorRegionCompare_symmetrical(self.average_speed, [5/1000,5/1000,.5/1000]) 
                    and self._vectorRegionCompare(self._as_array(self.current_wrench.wrench.force), [1.5,1.5,seeking_force*-.75], [-1.5,-1.5,seeking_force*-1.25])):
                    collision_confidence = collision_confidence + 1/self.rateSelected
                    rospy.logerr_throttle(.5, "Monitoring for flat surface, confidence = " + str(collision_confidence))
                    #if((rospy.Time.now()-marked_time).to_sec() > .50): #if we've satisfied this condition for 1 second
                    if(collision_confidence > .90):
                            #Stopped moving vertically and in contact with something that counters push force
                            rospy.logerr("Flat surface detected! Moving to spiral search!")
                            #Measure flat surface height:
                            surface_height = self.current_pose.transform.translation.z
                            state = 2
                            collision_confidence = 0.01
                else:
                    #rospy.logwarn_throttle(.5, "NOT a flat surface. Time: " + str((rospy.Time.now()-marked_time).to_sec()))
                    collision_confidence = np.max( np.array([collision_confidence * 95/self.rateSelected, .01]))
                    #marked_time = rospy.Time.now()
            elif (state == 2):
                #Spiral until we descend 1/3 the specified hole depth (provisional fraction)
                #This triggers the hole position estimate to be updated to limit crazy
                #forces and oscillations. Also reduces spiral size.
                seeking_force = 5.0
                wrench_vec  = self._get_command_wrench([0,0,seeking_force])
                pose_vec = self._spiral_search_basic_compliance_control()
                
                if(not self._force_cap_check()):
                    state = 99
                    rospy.logerr("Very high force/torque detected; going limp and ending test.")
                elif( self.current_pose.transform.translation.z <= surface_height - self.hole_depth/3):
                    collision_confidence = collision_confidence + 1/self.rateSelected
                    rospy.logerr_throttle(.5, "Monitoring for hole location, confidence = " + str(collision_confidence))
                    if(collision_confidence > .90):
                            #Descended from surface detection point. Updating hole location estimate.
                            self.x_pos_offset = self.current_pose.transform.translation.x
                            self.y_pos_offset = self.current_pose.transform.translation.y
                            self._amp_limit_cp = 2 * np.pi * 3 #limits to 3 spirals outward before returning to center.
                            #TODO - Make these runtime changes pass as parameters to the "spiral_search_basic_compliance_control" function
                            rospy.logerr_throttle(1.0, "Hole found, peg inserting...")
                            state = 3
                else:
                    collision_confidence = np.max( np.array([collision_confidence * 95/self.rateSelected, .01]))
                    if(self.current_pose.transform.translation.z >= surface_height - self.hole_depth):
                        rospy.logwarn_throttle(.5, "Height is still " + str(self.current_pose.transform.translation.z) 
                            + " whereas we should drop down to " + str(surface_height - self.hole_depth) )
            elif (state == 3):
                #Continue spiraling downward. Outward normal force is used to verify that the peg can't move
                #horizontally. We keep going until vertical speed is very near to zero.
                seeking_force = 5.0
                wrench_vec  = self._get_command_wrench([0,0,seeking_force])
                pose_vec = self._spiral_search_basic_compliance_control()
                
                if(not self._force_cap_check()):
                    state = 99
                    rospy.logerr("Very high force/torque detected; going limp and ending test.")
                elif( self._vectorRegionCompare_symmetrical(self.average_speed, [2.5/1000,2.5/1000,.5/1000]) 
                    and not self._vectorRegionCompare(self._as_array(self.current_wrench.wrench.force), [7,7,80], [-7,-7,-80])
                    and self.current_pose.transform.translation.z <= surface_height - self.hole_depth):
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
                    if(self.current_pose.transform.translation.z >= surface_height - self.hole_depth):
                        rospy.logwarn_throttle(.5, "Height is still " + str(self.current_pose.transform.translation.z) 
                            + " whereas we should drop down to " + str(surface_height - self.hole_depth) )
            elif (state == 4):
                #Inserted properly.
                rospy.logwarn_throttle(1.0, "Hole found, peg inserted! Done!")
                wrench_vec  = self._get_command_wrench([0,0,0])
                pose_vec = self._full_compliance_position()
                
            elif (state == 99):
                #Safety passivation; chill and pull out. Actually restarts itself if everything's chill enough.
                restart_height = .17
                if(self.current_pose.transform.translation.z > restart_height+.025):
                    #High enough, won't pull itself upward.
                    seeking_force = -2.5
                else:
                    #pull upward gently to move out of trouble hopefully.
                    seeking_force = -7
                wrench_vec  = self._get_command_wrench([0,0,seeking_force])
                pose_vec = self._full_compliance_position()
                restart_height = .17
                                
                rospy.logerr_throttle(.5, "Task suspended for safety. Freewheeling until low forces and height reset above .20: " + str(self.current_pose.transform.translation.z))

                if( self._vectorRegionCompare_symmetrical(self.average_speed, [.5/1000,.5/1000,5/1000]) 
                    and self._vectorRegionCompare_symmetrical(self._as_array(self.current_wrench.wrench.force), [1,1,5.5])
                    and self.current_pose.transform.translation.z > restart_height):
                    collision_confidence = collision_confidence + .25/self.rateSelected
                    rospy.logerr_throttle(.5, "Static. Restarting in " + str( np.round(4*(1-collision_confidence), 1) ) + " seconds...")
                    #if((rospy.Time.now()-marked_time).to_sec() > .50): #if we've satisfied this condition for 1 second
                    if(collision_confidence > 1):
                            #Stopped moving vertically and in contact with something that counters push force
                            rospy.logerr_throttle(1.0, "Restarting test!")
                            state = marked_state
                else:
                    collision_confidence = np.max( np.array([collision_confidence * 90/self.rateSelected, .01]))
                    if(self.current_pose.transform.translation.z > restart_height):
                        rospy.logwarn_throttle(.5, "That's high enough! Let robot stop and come to zero force.")

            self._publish_pose(pose_vec)
            self._publish_wrench(wrench_vec)
            self.rate.sleep()

#Stuck in hole:
# wrench [-1.50958516  1.74732935 -0.22236535]#      speed  [-1.45376413  1.48960552 -4.43557056]
# wrench [ 2.43184626  5.00212574 -2.4536103 ]#      speed  [-2.04941965  0.83368837  0.0782959 ]
# wrench [ 6.59614     6.07010813 -4.77999039]#      speed  [-1.65830209 -0.22519013 -0.5259787 ]
# wrench [11.7798602   5.16348097 -5.10239619]#      speed  [-1.45911821 -0.93610951  0.00294496]
# wrench [16.46678906  1.59219539 -4.69272811]#      speed  [-0.85717886 -1.4603617   0.2064414 ]
# wrench [17.90438252 -2.29216179 -5.00163983]#      speed  [-0.0339175  -1.86771153  0.03727609]
# wrench [17.90873177 -8.12120226 -4.66858722]#      speed  [ 1.03601238 -1.90853476  0.17106011]
# wrench [ 14.64670739 -12.45791743  -4.80226696]#   speed  [ 1.48294452 -1.45552691 -0.03383374]
# wrench [ 10.32158067 -15.00926006  -4.87042746]#   speed  [ 2.09561829 -0.52449749  0.10261508]
# wrench [  5.15586338 -15.11241063  -4.779649  ]#   speed  [ 1.90206149  0.48653684 -0.06437652]
# wrench [ -0.31361284 -13.19418616  -4.73827783]#   speed  [1.35331007 1.11014077 0.0984785 ]
# wrench [-3.51015968 -8.49405472 -4.90193306]#      speed  [ 0.88421147  1.98615908 -0.04157601]
# wrench [-4.72975108 -3.6191265  -4.56946905]       speed  [0.16434818 2.18659324 0.27017341]
# wrench [-2.77971526  1.80464577 -4.78332949]       speed  [-1.123056    1.66784535  0.0663183 ]


if __name__ == '__main__':
    rospy.init_node("demo_assembly_application_compliance")
    
    # assembly_application = PegInHoleNodeCompliance()
    # assembly_application._algorithm_force_control()

    #---------------------------------------------COMPLIANCE CONTROL BELOW, FORCE CONTROL ABOVE
    rospy.sleep(3.5)
    assembly_application = PegInHoleNodeCompliance()
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


