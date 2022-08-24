# Copyright 2021 Southwest Research Institute
# Licensed under the Apache License, Version 2.0

from conntact.assembly_algorithm_blocks import AlgorithmBlocks, AssemblyStep
from conntact.conntact_interface import ConntactInterface
from conntact.assembly_tools import AssemblyTools
import numpy as np
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
        transitions = [
            {'trigger':APPROACH_SURFACE_TRIGGER  , 'source':IDLE_STATE          , 'dest':APPROACH_STATE         },
            {'trigger':STEP_COMPLETE_TRIGGER     , 'source':APPROACH_STATE      , 'dest':FIND_HOLE_STATE        },
            {'trigger':FIND_HOLE_TRIGGER         , 'source':APPROACH_STATE      , 'dest':FIND_HOLE_STATE        },
            {'trigger':INSERT_PEG_TRIGGER        , 'source':FIND_HOLE_STATE     , 'dest':INSERTING_PEG_STATE    },
            {'trigger':ASSEMBLY_COMPLETED_TRIGGER, 'source':INSERTING_PEG_STATE , 'dest':COMPLETION_STATE       },
            {'trigger':SAFETY_RETRACTION_TRIGGER , 'source':'*'                 , 'dest':SAFETY_RETRACT_STATE, 'unless':'is_already_retracting' },
            {'trigger':RESTART_TEST_TRIGGER      , 'source':SAFETY_RETRACT_STATE, 'dest':APPROACH_STATE         },
            {'trigger':RUN_LOOP_TRIGGER      , 'source':'*', 'dest':None, 'after': 'run_loop'}
        ]
        self.steps:dict = { APPROACH_STATE: (FindSurface, []) }

        Machine.__init__(self, states=states, transitions=transitions, initial=IDLE_STATE)

        self.readYAML()
        self.tcp_selected = 'tip'

    def read_peg_hole_dimensions(self):
        """Read peg and hole data from YAML configuration file.
        """
        self.hole_depth = self.connfig['objects']['dimensions']['min_insertion_depth'] / 1000
        self.safe_clearance = self.connfig['objects']['dimensions']['safe_clearance'] / 2000;  # = .2 *radial* clearance i.e. on each side.

    def readYAML(self):
        """Read data from job config YAML and make certain calculations for later use. Stores peg frames in dictionary tool_data
        """

        # job parameters moved in from the peg_in_hole_params.yaml file
        # 'peg_4mm' 'peg_8mm' 'peg_10mm' 'peg_16mm'
        # 'hole_4mm' 'hole_8mm' 'hole_10mm' 'hole_16mm'
        # self.target_peg = self.connfig['task']['target_peg']
        # self.target_hole = self.connfig['task']['target_hole']
        activeTCP = self.connfig['task']['starting_tcp']

        self.read_board_positions()

        self.read_peg_hole_dimensions()

        # Spiral parameters
        self._spiral_params = self.connfig['task']['spiral_params']

        # Calculate transform from TCP to peg corner
        peg_locations = self.connfig['objects']['grasping_locations']

        # Set up tool_data.
        self.conntext.toolData.name = activeTCP
        self.conntext.toolData.validToolsDict = peg_locations

        self.conntext.select_tool(activeTCP)
        print("Select Tool successful!")

        self.surface_height = 0.0 # Starting height assumption
        self.restart_height = self.connfig['task']['restart_height']  # Height to restart

    def read_board_positions(self):
        """ Calculates pose of target hole relative to robot base frame.
        """
        self.target_hole_pose = self.interface.get_transform("target_hole_position", "base_link")
        # self.send_reference_TFs()
        self.x_pos_offset = self.target_hole_pose.transform.translation.x
        self.y_pos_offset = self.target_hole_pose.transform.translation.y

    def spiral_search_motion(self, frequency=.15, min_amplitude=.002, max_cycles=62.83185):
        """Generates position, orientation offset vectors which describe a plane spiral about z;
        Adds this offset to the current approach vector to create a searching pattern. Constants come from Init;
        x,y vector currently comes from x_ and y_pos_offset variables.
        """
        curr_time = self.interface.get_unified_time() - self._start_time
        curr_time_numpy = np.double(curr_time.to_sec())
        curr_amp = min_amplitude + self.safe_clearance * np.mod(2.0 * np.pi * frequency * curr_time_numpy, max_cycles);
        x_pos = curr_amp * np.cos(2.0 * np.pi * frequency * curr_time_numpy)
        y_pos = curr_amp * np.sin(2.0 * np.pi * frequency * curr_time_numpy)
        x_pos = x_pos + self.x_pos_offset
        y_pos = y_pos + self.y_pos_offset
        z_pos = self.current_pose.transform.translation.z
        pose_position = [x_pos, y_pos, z_pos]
        pose_orientation = [0, 1, 0, 0]  # w, x, y, z

        return [pose_position, pose_orientation]

    def finding_hole(self):
        #Spiral until we descend 1/3 the specified hole depth (provisional fraction)
        #This triggers the hole position estimate to be updated to limit crazy
        #forces and oscillations. Also reduces spiral size.

        seeking_force = -7.0
        self.wrench_vec  = self.conntext.get_command_wrench([0,0,seeking_force])
        self.pose_vec = self.spiral_search_motion(self.conntext._spiral_params["frequency"],
            self.conntext._spiral_params["min_amplitude"], self.conntext._spiral_params["max_cycles"])

        if(not self.conntext.force_cap_check(*self.cap_check_forces)):
            self.next_trigger, self.switch_state = self.post_action(SAFETY_RETRACTION_TRIGGER)
            rospy.logerr("Force/torque unsafe; pausing application.")
        elif( self.conntext.current_pose.transform.translation.z <= self.conntext.surface_height - .0004):
            #If we've descended at least 5mm below the flat surface detected, consider it a hole.
            self.completion_confidence = self.completion_confidence + 1/self.rate_selected
            rospy.loginfo_throttle(1, "Monitoring for hole location, confidence = " + str(self.completion_confidence))
            if(self.completion_confidence > .90):
                    #Descended from surface detection point. Updating hole location estimate.
                    self.conntext.x_pos_offset = self.conntext.current_pose.transform.translation.x
                    self.conntext.y_pos_offset = self.conntext.current_pose.transform.translation.y
                    self.conntext._amp_limit_cp = 2 * np.pi * 4 #limits to 3 spirals outward before returning to center.
                    #TODO - Make these runtime changes pass as parameters to the "spiral_search_basic_compliance_control" function
                    self.print("Hole found, peg inserting...")
                    self.next_trigger, self.switch_state = self.post_action(INSERT_PEG_TRIGGER)
        else:
            self.completion_confidence = np.max( np.array([self.completion_confidence * 95/self.rate_selected, .01]))

    def all_states_calc(self):
        self.publish_plotted_values(('state', self.state))
        super().all_states_calc()

    def publish_plotted_values(self, stateInfo) -> None:
        """Publishes critical data for plotting node to process.
        """

        items = dict()
        # Send a dictionary as plain text to expose some additional info
        items["status_dict"] = dict(
            {stateInfo, ('tcp_name', str(self.conntext.toolData.frame_name))})
        if (self.surface_height != 0.0):
            # If we have located the work surface
            items["status_dict"]['surface_height'] = str(self.surface_height)
        items["_average_wrench_world"] = self.conntext._average_wrench_world
        items["average_speed"] = self.conntext.average_speed
        items["current_pose"] = self.conntext.current_pose.transform.translation

        self.interface.publish_plotting_values(items)

    def main(self):
        self.algorithm_execute()
        self.interface.send_info("Spiral Search all done!")


class FindSurface(AssemblyStep):

    def __init__(self, algorithmBlocks: (AlgorithmBlocks)) -> None:
        AssemblyStep.__init__(self, algorithmBlocks)
        self.comply_axes = [0, 0, 1]
        self.seeking_force = [0, 0, -7]

    def exitConditions(self) -> bool:
        return self.static() and self.collision()

    def onExit(self):
        """Executed once, when the change-state trigger is registered.
        """
        # Measure flat surface height and report it to AssemblyBlocks:
        self.assembly.surface_height = self.conntext.current_pose.transform.translation.z
        return super().onExit()

class FindSurfaceFullCompliant(AssemblyStep):
    def __init__(self, algorithmBlocks: (AlgorithmBlocks)) -> None:
        AssemblyStep.__init__(self, algorithmBlocks)
        self.comply_axes = [1, 1, 1]
        self.seeking_force = [0, 0, -7]

    def exitConditions(self) -> bool:
        return self.static() and self.collision()

class SafetyRetraction(AssemblyStep):
    def __init__(self, algorithmBlocks: (AlgorithmBlocks)) -> None:
        AssemblyStep.__init__(self, algorithmBlocks)
        self.comply_axes = [1, 1, 1]
        self.seeking_force = [0, 0, 7]

    def exitConditions(self) -> bool:
        return self.noForce() and self.above_restart_height()

    def above_restart_height(self):
        return self.conntext.current_pose.transform.translation.z > \
               self.assembly.surface_height + self.assembly.reset_height


