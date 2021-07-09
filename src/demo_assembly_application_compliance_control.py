#!/usr/bin/env python

# Imports for ros
import rospy
# import tf
import numpy as np
from rospkg import RosPack
from geometry_msgs.msg import WrenchStamped, Wrench, TransformStamped, PoseStamped, Pose, Point, Quaternion, Vector3

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
        self._pose_pub = rospy.Publisher('cartesian_compliance_controller/target_frame', PoseStamped , queue_size=1)

        #Needed to get current pose of the robot
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        # self.tf_buffer = tf2.BufferCore(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self._seq = 0
        self._start_time = rospy.get_rostime() #for _spiral_search_basic_force_control and _spiral_search_basic_compliance_control
        self._freq = np.double(0.1) #Hz frequency in _spiral_search_basic_force_control
        self._amp  = np.double(10.0)  #Newton amplitude in _spiral_search_basic_force_control
        self._first_wrench = self._create_wrench([0,0,0], [0,0,0])


        self._freq_c = np.double(0.1) #Hz frequency in _spiral_search_basic_compliance_control
        self._amp_c  = np.double(.005)  #meters amplitude in _spiral_search_basic_compliance_control



    def _spiral_search_basic_compliance_control(self):
        curr_time = rospy.get_rostime() - self._start_time
        curr_time_numpy = np.double(curr_time.to_sec())

        # x_pos_offset = 0.88 #TODO:Assume the part needs to be inserted here at the offset. Fix with real value later
        # y_pos_offset = 0.550 #TODO:Assume the part needs to be inserted here at the offset. Fix with real value later
        x_pos_offset = 0.539 #TODO:Assume the part needs to be inserted here at the offset. Fix with real value later
        y_pos_offset = -0.238 #TODO:Assume the part needs to be inserted here at the offset. Fix with real value later

        # self._amp_c = self._amp_c * (curr_time_numpy * 0.001 * curr_time_numpy+ 1)

        x_pos = self._amp_c * np.cos(2.0 * np.pi * self._freq_c *curr_time_numpy)
        x_pos = x_pos + x_pos_offset

        y_pos = self._amp_c * np.sin(2.0 * np.pi * self._freq_c *curr_time_numpy)
        y_pos = y_pos + y_pos_offset


        # z_pos = 0.2 #0.104 is the approximate height of the hole itself. TODO:Assume the part needs to be inserted here. Update once I know the real value 
        z_pos = self._get_current_z_pos() #0.104 is the approximate height of the hole itself. TODO:Assume the part needs to be inserted here. Update once I know the real value 

        pose_position = [x_pos, y_pos, z_pos]

        pose_orientation = [0, 1, 0, 0] # w, x, y, z, TODO: Fix such that Jerit's code is not assumed correct. Is this right?

        return [pose_position, pose_orientation]

    def _get_current_z_pos(self):
        # transform = self.tf_buffer.lookup_transform("tool0",
        # "base_link", #source frame
        # # pose_stamped_to_transform.header.base_link, #source frame
        # rospy.get_rostime(), #get the tf at the current time
        # # rospy.Time(0), #get the tf at first available time
        # rospy.Duration(1.0)) #wait for 1 second
        # # pose_transform = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        
        transform = self.tf_buffer.lookup_transform("base_link", "tool0", rospy.Time(0), rospy.Duration(100.0))
        # print(transform)


        #return the z position only of the pose
        return transform.transform.translation.z

    def _get_command_wrench(self):
        curr_time = rospy.get_rostime() - self._start_time
        curr_time_numpy = np.double(curr_time.to_sec())

        # x_f = self._amp * np.cos(2.0 * np.pi * self._freq *curr_time_numpy)
        # y_f = self._amp * np.sin(2.0 * np.pi * self._freq *curr_time_numpy)
        x_f = 0
        y_f = 0
        z_f = 10.0 #apply constant downward force

        return [x_f, y_f, z_f, 0, 0, 0]


    def _publish_wrench(self, input_vec):
        # self.check_controller(self.force_controller)
        # forces, torques = self.com_to_tcp(result[:3], result[3:], transform)
        # result_wrench = self._create_wrench(result[:3], result[3:])
        # result_wrench = self._create_wrench([7,0,0], [0,0,0])
        result_wrench = self._create_wrench(input_vec[:3], input_vec[3:])
        
        self._wrench_pub.publish(result_wrench)
        

    # def _publish_pose(self, position, orientation):
    def _publish_pose(self, pose_stamped_vec):
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
        forces = sensor_wrench.wrench.force
        torques = sensor_wrench.wrench.torque
        # forces, torques = self.tcp_to_com(forces, torques)
        self._first_wrench = self._create_wrench([forces.x, forces.y, forces.z,
                        torques.x, torques.y, torques.z])




    def _algorithm_force_control(self):

        rate = rospy.Rate(50) #setup for sleeping at 10hz
        while not rospy.is_shutdown():           

            command_vec = self._spiral_search_basic_force_control()
            self._publish(command_vec)

            rate.sleep()

    def _algorithm_compliance_control(self):

        rate = rospy.Rate(500) #setup for sleeping in hz
        while not rospy.is_shutdown():       

            pose_vec = self._spiral_search_basic_compliance_control()
            wrench_vec  = self._get_command_wrench()
            self._publish_pose(pose_vec)
            self._publish_wrench(wrench_vec)


            rate.sleep()



if __name__ == '__main__':
    rospy.init_node("demo_assembly_application_compliance")
    
    # assembly_application = PegInHoleNodeCompliance()
    # assembly_application._algorithm_force_control()
    #---------------------------------------------COMPLIANCE CONTROL BELOW, FORCE CONTROL ABOVE

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


