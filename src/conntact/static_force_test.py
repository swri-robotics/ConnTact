# Copyright 2021 Southwest Research Institute
# Licensed under the Apache License, Version 2.0

#UR IP Address is now 175.31.1.137
#Computer has to be 175.31.1.150

# Imports for ros
# from _typeshed import StrPath

import csv
import sys
from builtins import staticmethod
from operator import truediv
from pickle import STRING
from threading import Lock
from typing import List

import matplotlib.pyplot as plt
import numpy as np
import rospy
# import tf2
import tf2_geometry_msgs
import tf2_py
import tf2_ros
import yaml
from colorama import Back, Fore, Style, init
from colorama.initialise import reset_all
# from sensor_msgs.msg import JointState
# from assembly_ros.srv import ExecuteStart, ExecuteRestart, ExecuteStop
from controller_manager_msgs.srv import (ListControllers, LoadController,
                                         SwitchController)
from geometry_msgs.msg import PoseStamped, TransformStamped, WrenchStamped
from rospkg import RosPack
from rospy.core import configure_logging
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from transitions import Machine

from conntact.assembly_algorithm_blocks import AlgorithmBlocks, AssemblyStep
# from conntact.assembly_algorithm_step import AssemblyStep

IDLE_STATE           = 'state_idle'
CHECK_FEEDBACK_STATE = 'state_check_load_cell_feedback'
APPROACH_STATE       = 'state_finding_surface'
RUN_TEST_STATE       = 'state_running_test'
RESET_STATE          = 'state_reset_test'
# FIND_HOLE_STATE      = 'state_finding_hole'
# INSERTING_PEG_STATE  = 'state_inserting_along_axis'
COMPLETION_STATE     = 'state_completed_insertion'
SAFETY_RETRACT_STATE = 'state_safety_retraction' 

#Trigger names
CHECK_FEEDBACK_TRIGGER     = 'check loadcell feedback'
APPROACH_SURFACE_TRIGGER   = 'start approach'
ALL_TESTS_COMPLETE_TRIGGER  = 'all tests complete'
# INSERT_PEG_TRIGGER         = 'hole found'
# ASSEMBLY_COMPLETED_TRIGGER = 'assembly completed'
STEP_COMPLETE_TRIGGER      = 'next step'
SAFETY_RETRACTION_TRIGGER  = 'retract to safety'
RESTART_TEST_TRIGGER       = 'restart test'
RUN_LOOP_TRIGGER           = 'run looped code'

class StaticForceTest(AlgorithmBlocks, Machine):

    def __init__(self):
        
        

        # Override Assembly Tools config variables
        ROS_rate = 100 #setup for sleeping in hz
        start_time = rospy.get_rostime() 

        AlgorithmBlocks.__init__(self, ROS_rate, start_time)

        #Override Alg Blocks config variables:
        states = [
            IDLE_STATE, 
            CHECK_FEEDBACK_STATE,
            APPROACH_STATE, 
            # FIND_HOLE_STATE, 
            # INSERTING_PEG_STATE, 
            
            #New states:
            RUN_TEST_STATE,
            RESET_STATE,

            COMPLETION_STATE, 
            SAFETY_RETRACT_STATE
        ]
        # TODO: Implement sub-task while loops as reflexive transitions according to https://github.com/pytransitions/transitions#reflexive-from-multiple-states
        #TODO: Replace warnings and errors with colorized info according to https://stackoverflow.com/questions/287871/how-to-print-colored-text-to-the-terminal/3332860#3332860
        # TODO: Replace all transitions with dest "safety_retract_state" with a single one using 'source':'*'
        transitions = [
            {'trigger':CHECK_FEEDBACK_TRIGGER    , 'source':IDLE_STATE          , 'dest':CHECK_FEEDBACK_STATE   },
            {'trigger':APPROACH_SURFACE_TRIGGER  , 'source':CHECK_FEEDBACK_STATE, 'dest':APPROACH_STATE         },
            {'trigger':STEP_COMPLETE_TRIGGER     , 'source':APPROACH_STATE      , 'dest':RUN_TEST_STATE         },
            {'trigger':STEP_COMPLETE_TRIGGER     , 'source':RUN_TEST_STATE      , 'dest':RESET_STATE            },
            {'trigger':STEP_COMPLETE_TRIGGER     , 'source':RESET_STATE         , 'dest':APPROACH_STATE         },
            {'trigger':ALL_TESTS_COMPLETE_TRIGGER, 'source':RESET_STATE         , 'dest':COMPLETION_STATE       },
            {'trigger':RESTART_TEST_TRIGGER      , 'source':SAFETY_RETRACT_STATE, 'dest':RESET_STATE            },
            {'trigger':RUN_LOOP_TRIGGER          , 'source':'*'                 , 'dest':None, 'after': 'run_loop'},
            {'trigger':SAFETY_RETRACTION_TRIGGER , 'source':'*'                 , 'dest':SAFETY_RETRACT_STATE, 'unless':'is_already_retracting' }

        ]

        self.paramList = [
            {
                "name"          : "Test1",
                "axis"          : [0,0,1],
                "force"         : [0,0,-5],
                "stability"     : .90,
                "holdTime"      : 10
            },{
                "name"          : "Test2",
                "axis"          : [0,0,1],
                "force"         : [0,0,-10],
                "stability"     : .90,
                "holdTime"      : 5
            },{
                "name"          : "Test3",
                "axis"          : [0,0,1],
                "force"         : [0,0,-15],
                "stability"     : .90,
                "holdTime"      : 5
            },{
                "name"          : "Test4_high_force",
                "axis"          : [0,0,1],
                "force"         : [0,0,-20],
                "stability"     : .85,
                "holdTime"      : 5
            }

        ]

        self.target_position = self.target_hole_pose
        # rospy.logerr("Target hole position: " + str(self.target_position))
        self.testingStage = 0

        self.steps : dict = { 
            APPROACH_STATE: (ApproachStep, []) ,
            RUN_TEST_STATE: (RunTestStep, []) ,
            RESET_STATE: (ResetStep, [])            
            }

        Machine.__init__(self, states=states, transitions=transitions, initial=IDLE_STATE)

        self.updateTestParams()
        self.tcp_selected = 'tip'

    def updateTestParams(self):
        #Loads the next parameter set into the Steps dictionary in time for it to be run by the AlgorithmBlocks base class.
        rospy.logerr("Updating Params")
        self.steps[RUN_TEST_STATE] = (self.steps[RUN_TEST_STATE][0], [self.paramList[self.testingStage]])

    def main(self):
        # TODO: Remove following Sleep, use a rospy.wait command of some kind
        rospy.sleep(3)
        rospy.wait_for_message("cartesian_compliance_controller/ft_sensor_wrench", WrenchStamped)
        
        self.algorithm_execute()
        rospy.loginfo("Static Force Test all done!")

if __name__ == '__main__':
    
    assembly_application = StaticForceTest()

    assembly_application.main()
    
class ApproachStep(AssemblyStep):
    def __init__(self, algorithmBlocks:AlgorithmBlocks, paramsDict = {
                "axis"          : [0,0,1],
                "force"         : [0,0,-7],
                "stability"     : .95,
                "holdTime"      : 1
            }) -> None:
        AssemblyStep.__init__(self, algorithmBlocks)
        #set up the parameters for this step
        self.paramsDict=paramsDict
        # self.desiredOrientation = trfm.quaternion_from_euler(0,0,-90)
        #Set up exit condition sensitivity
        self.seeking_force = self.paramsDict["force"]#Using seekingForce syncs the collision detection check with the actual force command
        self.exitPeriod = self.paramsDict["holdTime"]      #Seconds to stay within bounds
        self.holdStartTime = 0;
        self.exitThreshold = self.paramsDict["stability"]    #Percentage of time for the last period

    def execute(self):
        # rospy.logerr_throttle(2, Fore.BLUE +"Approach step executing!" + Style.RESET_ALL)
        self.updateCommands()
        
    def updateCommands(self):
        # rospy.loginfo_throttle(1, Fore.BLUE + 'Updating commands ' + Style.RESET_ALL)
        #Command wrench
        self.assembly.wrench_vec  = self.assembly.get_command_wrench(self.seeking_force)
        #Command pose
        # self.assembly.pose_vec = self.assembly.linear_search_position([0,0,0])
        self.assembly.pose_vec = self.assembly.arbitrary_axis_comply(self.paramsDict["axis"])

        # rospy.loginfo_throttle(2, Fore.CYAN + " Arbitrary command wrench is giving \n" + str(self.assembly.pose_vec) + "\nwhereas linear search would give \n" + str(self.assembly.linear_search_position([0,0,0])))

    def exitConditions(self) -> bool:
        return self.collision() and self.static()

    def onExit(self):
            rospy.logerr("Moving to next step!")
            
            #Measure flat surface height and store it for future steps:
            #TODO: Generalize the "Saving data for later steps" capability so it doesn't have to be a bunch of messy class variables.
            self.assembly.surface_height = self.assembly.current_pose.transform.translation.z
            #Tell the state machine to move to the next step according to the transitions dictionary. This way the AssemblyStep class doesn't need the information of its next state. You can have it send Triggers to go to other states instead.
            return STEP_COMPLETE_TRIGGER, True

class RunTestStep(ApproachStep):
    def __init__(self, algorithmBlocks:AlgorithmBlocks, paramsDict = {
                "name"          : "Test",
                "axis"          : [0,0,1],
                "force"         : [0,0,-7],
                "stability"     : .95,
                "holdTime"      : 5
            }) -> None:
        ApproachStep.__init__(self, algorithmBlocks, paramsDict)

        rospy.loginfo(Fore.BLUE+ "Executing test according to configuration: "+str(self.paramsDict["name"])+ Style.RESET_ALL)
        self.loggingInterval = rospy.Time.from_sec(1/20) #Try to log at 20 hz for now.
        self.now = self.startTime = self.lastLog = rospy.Time.now()#record starting time
        self.file = self.openCSV(sys.path[0]+'/output_data' + self.paramsDict["name"] + '.csv')
        self.outputter = csv.writer(self.file)
        self.writeLine(["Test number " + str(self.assembly.testingStage), "Today's date: TODO"])
        self.writeLine(["Time", "Force X", "Force Y", "Force Z", "Position X", "Position Y", "Position Z"])

        # The system should time out if it's not stable after some period. I'm going to say 4 times the hold time.
        self.timeoutDuration = rospy.Duration(self.paramsDict["holdTime"] * 3)
        

    def execute(self):
        self.logData()
        return super().execute()

    def onExit(self):
        rospy.logerr("Test complete! Resetting... ")
        self.closeCSV()
        #Tell the state machine to move to the next step according to the transitions dictionary. This way the AssemblyStep class doesn't need the information of its next state. You can have it send Triggers to go to other states instead.
        return STEP_COMPLETE_TRIGGER, True

    def openCSV(self, path = sys.path[0]+'/output_data.csv'):
        rospy.loginfo(Fore.MAGENTA + "CSV: Opening! Path is: "+str(path)+ Style.RESET_ALL)
        return open(path, "w", newline='')

    def writeLine(self, line:list):
        self.outputter.writerow(line)

    def exitConditions(self) -> bool:
        return self.static() and self.collision()

    def checkCompletion(self):
        if((self.now - self.startTime) > self.timeoutDuration):
            rospy.loginfo(Fore.MAGENTA + "Execution step unstable, timeout reached." + Style.RESET_ALL)
            self.writeLine(["Unstable! Test timed out.", "Exit time: ", str(self.timeoutDuration)])
        return super().checkCompletion()

    def logData(self):
        self.now = rospy.Time.now()
        if((self.now - self.loggingInterval).to_sec() > self.lastLog.to_sec()):
            #It's been long enough, log data again.
            self.lastLog = rospy.Time.now()
            force = self.assembly._average_wrench_world.force
            pos = self.assembly.current_pose.transform.translation
            self.writeLine([str(rospy.get_time()),  force.x, force.y, force.z, pos.x, pos.y, pos.z])
        
    def closeCSV(self):
        rospy.loginfo(Fore.MAGENTA + "CSV: Closing!"+ Style.RESET_ALL)
        self.file.close()


class ResetStep(AssemblyStep):
        def __init__(self, algorithmBlocks:AlgorithmBlocks, paramsDict = {
                    "axis"          : [0,0,1],
                    "force"         : [0,0,20],
                    "stability"     : .90,
                    "holdTime"      : .25
                }) -> None:
            AssemblyStep.__init__(self, algorithmBlocks)
            #set up the parameters for this step
            self.paramsDict=paramsDict
            # self.desiredOrientation = trfm.quaternion_from_euler(0,0,-90)
            #Set up exit condition sensitivity
            self.seekingForce = self.paramsDict["force"]
            self.exitPeriod = self.paramsDict["holdTime"]      #Seconds to stay within bounds
            self.holdStartTime = 0;
            self.exitThreshold = self.paramsDict["stability"]    #Percentage of time for the last period

        def execute(self):
            # rospy.logerr_throttle(2, Fore.BLUE +"Reset step executing!" + Style.RESET_ALL)
            self.updateCommands()
            
        def updateCommands(self):
            # rospy.loginfo_throttle(1, Fore.BLUE + 'Updating commands ' + Style.RESET_ALL)
            #Command wrench
            self.assembly.wrench_vec  = self.assembly.get_command_wrench(self.seeking_force)
            #Command pose
            # self.assembly.pose_vec = self.assembly.linear_search_position([0,0,0])
            self.assembly.pose_vec = self.assembly.arbitrary_axis_comply(self.paramsDict["axis"])

            # rospy.loginfo_throttle(2, Fore.CYAN + " Arbitrary command wrench is giving \n" + str(self.assembly.pose_vec) + "\nwhereas linear search would give \n" + str(self.assembly.linear_search_position([0,0,0])))

        def exitConditions(self) -> bool:
            return self.awayFromSurface() and self.noForce()

        def awayFromSurface(self)->bool:
            #Check if we've retreated 1cm from the surface
            currentHeight = self.assembly.current_pose.transform.translation.z

            return (currentHeight - self.assembly.surface_height > -.05)

        def onExit(self):
                if(self.assembly.testingStage < len(self.assembly.paramList) ):
                    self.assembly.testingStage += 1
                    self.assembly.updateTestParams()
                    rospy.logerr("Reset successful! Moving to test " + str(self.assembly.testingStage) )
                    return STEP_COMPLETE_TRIGGER, True
                else:
                    rospy.logerr("Last test complete, quitting! " + str(self.assembly.testingStage) )
                    return ALL_TESTS_COMPLETE_TRIGGER, True
                #Measure flat surface height and store it for future steps:
                #TODO: Generalize the "Saving data for later steps" capability so it doesn't have to be a bunch of messy class variables.
                #Tell the state machine to move to the next step according to the transitions dictionary. This way the AssemblyStep class doesn't need the information of its next state. You can have it send Triggers to go to other states instead.
                

