#!/usr/bin/env python

#UR IP Address is now 175.31.1.137
#Computer has to be 175.31.1.150

# Imports for ros
# from _typeshed import StrPath

from builtins import staticmethod
from operator import truediv
from pickle import STRING

from colorama.initialise import reset_all
import rospy
import sys
import numpy as np
import matplotlib.pyplot as plt
from rospkg import RosPack
from geometry_msgs.msg import WrenchStamped, Wrench, TransformStamped, PoseStamped, Pose, Point, Quaternion, Vector3, Transform
from rospy.core import configure_logging

from colorama import Fore, Back, Style, init
# from sensor_msgs.msg import JointState
# from assembly_ros.srv import ExecuteStart, ExecuteRestart, ExecuteStop
from controller_manager_msgs.srv import SwitchController, LoadController, ListControllers
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

import tf2_ros
import tf2_py 
# import tf2
import tf2_geometry_msgs


from threading import Lock

from peg_in_hole_demo.assembly_algorithm_blocks import AlgorithmBlocks

from transitions import Machine


class testing():
    def __init__(self):
        self._wrench_pub = rospy.Publisher('/cartesian_compliance_controller/target_wrench', WrenchStamped, queue_size=10)

class CornerSearch(AlgorithmBlocks, Machine):

    def __init__(self):
        ROS_rate = 100 #setup for sleeping in hz
        start_time = rospy.get_rostime() 
        super.__init__(self, ROS_rate, start_time)

        self.tcp_selected = 'corner'

    def main(self):
        # TODO: Remove following Sleep, use a rospy.wait command of some kind
        rospy.sleep(3)
        rospy.wait_for_message("cartesian_compliance_controller/target_wrench", WrenchStamped)
        
        self.algorithm_execute()
        rospy.loginfo("Corner Search all done!")

if __name__ == '__main__':
    
    assembly_application = CornerSearch()
    assembly_application.main()
    
