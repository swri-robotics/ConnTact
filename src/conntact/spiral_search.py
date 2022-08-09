# Copyright 2021 Southwest Research Institute
# Licensed under the Apache License, Version 2.0

# Imports for ros

import rospy
from geometry_msgs.msg import WrenchStamped, Wrench, TransformStamped, PoseStamped, Pose, Point, Quaternion, Vector3, Transform
from conntact.assembly_algorithm_blocks import AlgorithmBlocks, AssemblyStep
from conntact.ros_conntact_interface import ConntactROSInterface
from transitions import Machine

IDLE_STATE           = 'state_idle'
CHECK_FEEDBACK_STATE = 'state_check_load_cell_feedback'
APPROACH_STATE       = 'state_finding_surface'
FIND_HOLE_STATE      = 'state_finding_hole'
INSERTING_PEG_STATE  = 'state_inserting_along_axis'
COMPLETION_STATE     = 'state_completed_insertion'
EXIT_STATE           = 'state_exit'
SAFETY_RETRACT_STATE = 'state_safety_retraction' 


#Trigger names
CHECK_FEEDBACK_TRIGGER     = 'check loadcell feedback'
APPROACH_SURFACE_TRIGGER   = 'start approach'
FIND_HOLE_TRIGGER          = 'surface found'
INSERT_PEG_TRIGGER         = 'hole found'
ASSEMBLY_COMPLETED_TRIGGER = 'assembly completed'
STEP_COMPLETE_TRIGGER      = 'next step'
SAFETY_RETRACTION_TRIGGER  = 'retract to safety'
RESTART_TEST_TRIGGER       = 'restart test'
RUN_LOOP_TRIGGER           = 'run looped code'


class SpiralSearch(AlgorithmBlocks, Machine):

    def __init__(self, ROS_rate, conntext, interface):

        AlgorithmBlocks.__init__(self, ROS_rate, conntext, interface)

        #Override Alg Blocks config variables:
        states = [
            IDLE_STATE, 
            CHECK_FEEDBACK_STATE,
            APPROACH_STATE, 
            FIND_HOLE_STATE, 
            INSERTING_PEG_STATE, 
            COMPLETION_STATE, 
            SAFETY_RETRACT_STATE
        ]
        # TODO: Implement sub-task while loops as reflexive transitions according to https://github.com/pytransitions/transitions#reflexive-from-multiple-states
        #TODO: Replace warnings and errors with colorized info according to https://stackoverflow.com/questions/287871/how-to-print-colored-text-to-the-terminal/3332860#3332860
        # TODO: Replace all transitions with dest "safety_retract_state" with a single one using 'source':'*'
        transitions = [
            {'trigger':CHECK_FEEDBACK_TRIGGER    , 'source':IDLE_STATE          , 'dest':CHECK_FEEDBACK_STATE   },
            {'trigger':APPROACH_SURFACE_TRIGGER  , 'source':CHECK_FEEDBACK_STATE, 'dest':APPROACH_STATE         },
            {'trigger':STEP_COMPLETE_TRIGGER     , 'source':APPROACH_STATE      , 'dest':FIND_HOLE_STATE        },
            {'trigger':FIND_HOLE_TRIGGER         , 'source':APPROACH_STATE      , 'dest':FIND_HOLE_STATE        },
            {'trigger':INSERT_PEG_TRIGGER        , 'source':FIND_HOLE_STATE     , 'dest':INSERTING_PEG_STATE    },
            {'trigger':ASSEMBLY_COMPLETED_TRIGGER, 'source':INSERTING_PEG_STATE , 'dest':COMPLETION_STATE       },
            {'trigger':SAFETY_RETRACTION_TRIGGER , 'source':'*'                 , 'dest':SAFETY_RETRACT_STATE, 'unless':'is_already_retracting' },
            {'trigger':RESTART_TEST_TRIGGER      , 'source':SAFETY_RETRACT_STATE, 'dest':APPROACH_STATE         },
            {'trigger':RUN_LOOP_TRIGGER      , 'source':'*', 'dest':None, 'after': 'run_loop'}

        ]

        Machine.__init__(self, states=states, transitions=transitions, initial=IDLE_STATE)

        self.tcp_selected = 'tip'

    def main(self):
        # TODO: Remove following Sleep, use a rospy.wait command of some kind
        rospy.sleep(3)
        rospy.wait_for_message("cartesian_compliance_controller/ft_sensor_wrench", WrenchStamped)
        
        self.algorithm_execute()
        rospy.loginfo("Spiral Search all done!")

if __name__ == '__main__':
    
    assembly_application = SpiralSearch()

    assembly_application.main()
    
