# Copyright 2021 Southwest Research Institute
# Licensed under the Apache License, Version 2.0

# Imports for ros
import rospy
# import tf
import numpy as np
from rospkg import RosPack
from geometry_msgs.msg import WrenchStamped, Wrench, TransformStamped, PoseStamped, Pose, Point, Quaternion, Vector3

from sensor_msgs.msg import JointState
# from assembly_ros.srv import ExecuteStart, ExecuteRestart, ExecuteStop
from controller_manager_msgs.srv import SwitchController, LoadController, ListControllers

from threading import Lock

class PegInHoleNode():

    def __init__(self):
        self._force_controller_pub = rospy.Publisher('/cartesian_force_controller/target_wrench', WrenchStamped, queue_size=10)
        # self._ft_sensor_sub = rospy.Subscriber('/cartesian_force_controller/ft_sensor_wrench', WrenchStamped, self._ft_sensor_callback)
        self._seq = 0
        self._start_time = rospy.get_rostime() #for _spiral_search_basic_force_control and _spiral_search_basic_compliance_control
        self._freq = np.double(0.2) #frequency in _spiral_search_basic_force_control
        self._amp  = np.double(5.0)  #amplitude in _spiral_search_basic_force_control
        self._first_wrench = self._create_wrench([0,0,0], [0,0,0])


        self._freq_c = np.double(0.1) #Hz frequency in _spiral_search_basic_compliance_control
        self._amp_c  = np.double(.010)  #meters amplitude in _spiral_search_basic_compliance_control



    def _spiral_search_basic_force_control(self):
        curr_time = rospy.get_rostime() - self._start_time
        curr_time_numpy = np.double(curr_time.to_sec())

        x_f = self._amp * np.cos(2.0 * np.pi * self._freq *curr_time_numpy)
        y_f = self._amp * np.sin(2.0 * np.pi * self._freq *curr_time_numpy)
        z_f = 10.0 #apply constant downward force

        return [x_f, y_f, z_f, 0, 0, 0]

    def _spiral_search_basic_compliance_control(self):
        curr_time = rospy.get_rostime() - self._start_time
        curr_time_numpy = np.double(curr_time.to_sec())

        x_f = self._amp * np.cos(2.0 * np.pi * self._freq *curr_time_numpy)
        y_f = self._amp * np.sin(2.0 * np.pi * self._freq *curr_time_numpy)
        z_f = 10.0 #apply constant downward force

        return [x_f, y_f, z_f, 0, 0, 0]


    def _publish(self, input_vec):
        # self.check_controller(self.force_controller)
        # forces, torques = self.com_to_tcp(result[:3], result[3:], transform)
        # result_wrench = self._create_wrench(result[:3], result[3:])
        # result_wrench = self._create_wrench([7,0,0], [0,0,0])
        result_wrench = self._create_wrench(input_vec[:3], input_vec[3:])
        
        self._force_controller_pub.publish(result_wrench)
        
        
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
        # Update currently observed forces
        forces = sensor_wrench.wrench.force
        torques = sensor_wrench.wrench.torque
        # forces, torques = self.tcp_to_com(forces, torques)
        self._first_wrench = self._create_wrench([forces.x, forces.y, forces.z,
                        torques.x, torques.y, torques.z])
        


        # self.set_wrench_obs([forces.x, forces.y, forces.z,
                        # torques.x, torques.y, torques.z])



    def _algorithm(self):

        rate = rospy.Rate(500) #setup for sleeping at 500hz
        while not rospy.is_shutdown():
            # self._force_controller_pub._publish(result_wrench)
            # print('WE HAVE PUBLISHED...WAITING FOR ROBOT TO MOVE...')
            

            command_vec = self._spiral_search_basic_force_control()
            self._publish(command_vec)

            rate.sleep()



if __name__ == '__main__':
    rospy.init_node("demo_assembly_application")
    
    assembly_application = PegInHoleNode()
    assembly_application._algorithm()


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


