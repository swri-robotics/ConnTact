# Copyright 2021 Southwest Research Institute
# Licensed under the Apache License, Version 2.0

#UR IP Address is now 175.31.1.137
#Computer has to be 175.31.1.150

import string
import sys
from builtins import staticmethod

import numpy as np
import rospy
import tf.transformations as trfm
from colorama import Back, Fore, Style, init

from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion, Transform,
                               TransformStamped, Vector3, Wrench,
                               WrenchStamped)

from transitions import Machine

from conntact.assembly_tools import AssemblyTools

"""State names
For loop purposes, the state name *must* be identical to "state_"+(loop method name)
i.e. in state "state_finding_surface" the method "finding_surface" will be run repeatedly; 
if it does not exist, an error will occur.
"""
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
SAFETY_RETRACTION_TRIGGER  = 'retract to safety'
RESTART_TEST_TRIGGER       = 'restart test'
STEP_COMPLETE_TRIGGER      = 'next step'
RUN_LOOP_TRIGGER           = 'run looped code'



# class AlgorithmBlocks(AssemblyTools):
class AlgorithmBlocks():

    def __init__(self, conntext, interface, connfig_name):
        self.conntext = conntext
        self.rate_selected = conntext.rate
        if interface is None:
            self.interface = self.conntext.interface
        else:
            self.interface = interface

        self.connfig = self.interface.load_yaml_file(connfig_name)
        self.start_time = self.interface.get_unified_time()

        #Configuration variables, to be moved to a yaml file later:
        self.pose_vec = None
        self.wrench_vec = self.conntext.get_command_wrench([0, 0, 0])

        self.next_trigger = ''  # Empty to start. Each callback should decide what next trigger to implement in the main loop

        #List the official states here. Takes strings, but the tokens created above so typos are less likely from repeated typing of strings (unchecked by interpreter).
        states = [
            IDLE_STATE,
            CHECK_FEEDBACK_STATE,
            APPROACH_STATE,
            FIND_HOLE_STATE,
            INSERTING_PEG_STATE,
            COMPLETION_STATE,
            EXIT_STATE,
            SAFETY_RETRACT_STATE
        ]

        #Define the valid transitions from/to each state. Here's where you define the functionality of the state machine. The system executes the first transition in this list which matches BOTH the trigger AND the CURRENT state.
        transitions = [
            {'trigger':CHECK_FEEDBACK_TRIGGER    , 'source':IDLE_STATE          , 'dest':CHECK_FEEDBACK_STATE   },
            {'trigger':APPROACH_SURFACE_TRIGGER  , 'source':CHECK_FEEDBACK_STATE, 'dest':APPROACH_STATE         },
            {'trigger':FIND_HOLE_TRIGGER         , 'source':APPROACH_STATE      , 'dest':FIND_HOLE_STATE        },
            {'trigger':INSERT_PEG_TRIGGER        , 'source':FIND_HOLE_STATE     , 'dest':INSERTING_PEG_STATE    },
            {'trigger':ASSEMBLY_COMPLETED_TRIGGER, 'source':INSERTING_PEG_STATE , 'dest':COMPLETION_STATE       },
            {'trigger':SAFETY_RETRACTION_TRIGGER , 'source':'*'                 , 'dest':SAFETY_RETRACT_STATE, 'unless':'is_already_retracting' },
            {'trigger':RESTART_TEST_TRIGGER      , 'source':SAFETY_RETRACT_STATE, 'dest':CHECK_FEEDBACK_STATE   },
            {'trigger':RUN_LOOP_TRIGGER      , 'source':'*', 'dest':None, 'after': 'run_loop'}

        ]

        self.previousState = None #Store a reference to the previous state here.
        # self.conntext.surface_height = 0.0

        Machine.__init__(self, states=states, transitions=transitions, initial=IDLE_STATE)


        # AssemblyTools.__init__(self, interface, ROS_rate)
        # Set up Colorama for colorful terminal outputs on all platforms
        init(autoreset=True)
        #Store a reference to the AssemblyStep class in use by the current State if it exists:
        self.step:AssemblyStep = None

    def print(self, string: str):
        self.interface.send_info(Fore.LIGHTBLUE_EX + string + Style.RESET_ALL)

    def post_action(self, trigger_name):
        """Defines the next trigger which the state machine should execute.
        """
        return [trigger_name, True]

    def is_already_retracting(self):
        return self.is_state_safety_retraction()
    def on_enter_state_finding_surface(self):
        self._log_state_transition()
    def on_enter_state_finding_hole(self):
        self._log_state_transition()
    def on_enter_state_inserting_along_axis(self):
        self._log_state_transition()
    def on_enter_state_completed_insertion(self):
        self._log_state_transition()
    def on_enter_state_retracting_to_safety(self):
        self._log_state_transition()


    def _log_state_transition(self):
        rospy.loginfo(Fore.BLACK + Back.WHITE +"State transition to " +
                      str(self.state) + " at time = " + str(self.interface.get_unified_time()) + Style.RESET_ALL )
    def reset_on_state_enter(self):
        self.completion_confidence = 0

    def update_commands(self):
        self.conntext.publish_pose(self.pose_vec)
        self.conntext.publish_wrench(self.wrench_vec)

    def run_loop(self):
        """Runs the method with name matching the state name. Superceded by AssemblyStep class type if one exists.
        """
        state_name=str(self.state)
        if("state_") in state_name:
            if(state_name in self.steps):
                #This step has been realized as a Step class
                if(not self.step):
                    #Set step to an instance of the referred class and pass in the parameters.
                    self.step = self.steps[state_name][0](self, *self.steps[state_name][1])
                    rospy.loginfo( Fore.GREEN + "Created step object " + str(type(self.step)) + Style.RESET_ALL )
                else:
                    self.step.execute()
                if (self.step.checkCompletion()):
                    self.next_trigger, self.switch_state = self.step.onExit()

            else:
                # This step has been realized as a looping method.
                method_name = "self."+state_name[state_name.find('state_')+6:]+'()'
                try:
                    rospy.loginfo_throttle(2, Fore.WHITE + "In state "+state_name + Style.RESET_ALL + ", now executing " + method_name)
                    exec(method_name)
                except (NameError, AttributeError):
                    rospy.logerr_throttle(2, "State name " + method_name + " does not match 'state_'+(state loop method name) in algorithm!")
                    pass    
                except:
                    print("Unexpected error when trying to locate state loop name:", sys.exc_info()[0])
                    raise
        else:
            rospy.logerr("Invalid state name! Terminating.")
            quit()
        
    def algorithm_execute(self):
        """Main execution loop. A True exit state will cause the buffered Trigger "self.next_trigger" to be run, changing the state. If using a state realized as an AssemblyStep class, we delete the old step here. Also executes the once-per-cycle non-step commands needed for continuous safe operation.
        """

        self.completion_confidence = 0
        
        self.next_trigger, self.switch_state = self.post_action(APPROACH_SURFACE_TRIGGER)

        rospy.loginfo(Fore.BLACK + Back.GREEN + "Beginning search algorithm. "+Style.RESET_ALL)

        while not rospy.is_shutdown() and self.state != EXIT_STATE:
            # Main program loop. 
            # Refresh values, run process trigger (either loop-back to perform state actions or transition to new state), then output controller commands and wait for next loop time.
            self.all_states_calc()
            self.checkForceCap()

            if(self.switch_state): #If the command to change states has come in:
                self.switch_state = False
                if(self.step):
                    #If a Step class has been defined for the current State, we delete it to be tidy.
                    del self.step
                    self.step = None
            else:
                # If we're not switching states, we use RUN_LOOP_TRIGGER to execute this state's loop code.
                self.next_trigger = RUN_LOOP_TRIGGER
                
            # Execute the trigger chosen
            self.trigger(self.next_trigger)

            # Publish robot motion commands only once per loop, right at the end of the loop:
            self.update_commands()
            self.interface.sleep_until_next_loop()
    #
    # def inserting_along_axis(self):
    #     #Continue spiraling downward. Outward normal force is used to verify that the peg can't move
    #     #horizontally. We keep going until vertical speed is very near to zero.
    #
    #
    #     seeking_force = -5.0
    #     self.wrench_vec  = self.conntext.get_command_wrench([0,0,seeking_force])
    #     self.pose_vec = self.conntext.full_compliance_position()
    #
    #     if(not self.conntext.force_cap_check(*self.cap_check_forces)):
    #         self.next_trigger, self.switch_state = self.post_action(SAFETY_RETRACTION_TRIGGER)
    #         rospy.logerr("Force/torque unsafe; pausing application.")
    #     elif( self.conntext.vectorRegionCompare_symmetrical(self.conntext.average_speed, self.speed_static)
    #         #and not self.conntext.vectorRegionCompare(self.conntext.as_array(self.conntext._average_wrench_world.force), [6,6,80], [-6,-6,-80])
    #         and self.conntext.vectorRegionCompare(self.conntext.as_array(self.conntext._average_wrench_world.force), [1.5,1.5,seeking_force*-1.5], [-1.5,-1.5,seeking_force*-.75])
    #         and self.conntext.current_pose.transform.translation.z <= self.conntext.surface_height - self.conntext.hole_depth):
    #         self.completion_confidence = self.completion_confidence + 1/self.rate_selected
    #         rospy.loginfo_throttle(1, "Monitoring for peg insertion, confidence = " + str(self.completion_confidence))
    #         #if((rospy.Time.now()-marked_time).to_sec() > .50): #if we've satisfied this condition for 1 second
    #         if(self.completion_confidence > .90):
    #                 #Stopped moving vertically and in contact with something that counters push force
    #                 self.next_trigger, self.switch_state = self.post_action(ASSEMBLY_COMPLETED_TRIGGER)
    #     else:
    #         #
    #         self.completion_confidence = np.max( np.array([self.completion_confidence * 95/self.rate_selected, .01]))
    #         if(self.conntext.current_pose.transform.translation.z >= self.conntext.surface_height - self.conntext.hole_depth):
    #             rospy.loginfo_throttle(1, Fore.YELLOW + "Height is still " + str(self.conntext.current_pose.transform.translation.z)
    #                 + " whereas we should drop down to " + str(self.conntext.surface_height - self.conntext.hole_depth) + Style.RESET_ALL)
    #
    # def completed_insertion(self):
    #     #Inserted properly.
    #
    #     rospy.loginfo_throttle(1, Fore.RED + "Hole found, peg inserted! Done!" +Style.RESET_ALL)
    #     if(self.conntext.current_pose.transform.translation.z > self.conntext.restart_height+.02):
    #         #High enough, won't pull itself upward.
    #         seeking_force = 5
    #         rospy.loginfo_once(Back.GREEN + Fore.WHITE + Style.BRIGHT + "Completed Task!" + Style.RESET_ALL)
    #         quit()
    #     else:
    #         #pull upward gently to move out of trouble hopefully.
    #         seeking_force = 20
    #     self.conntext.force_cap_check(*self.cap_check_forces)
    #     self.wrench_vec  = self.conntext.get_command_wrench([0,0,seeking_force])
    #     self.pose_vec = self.full_compliance_position()
    #
    # def safety_retraction(self):
    #     #Safety passivation; chill and pull out. Actually restarts itself if everything's chill enough.
    #
    #
    #     if(self.conntext.current_pose.transform.translation.z > self.conntext.restart_height+.05):
    #         #High enough, won't pull itself upward.
    #         seeking_force = 3.5
    #     else:
    #         #pull upward gently to move out of trouble.
    #         seeking_force = 10
    #     self.wrench_vec  = self.conntext.get_command_wrench([0,0,seeking_force])
    #     self.pose_vec = self.full_compliance_position()
    #
    #     rospy.loginfo_throttle(1, Fore.RED + "Task suspended for safety. Freewheeling until low forces and height reset above " + str(self.conntext.restart_height) + ': ' + str(self.conntext.current_pose.transform.translation.z) + Style.RESET_ALL)
    #     if( self.conntext.vectorRegionCompare_symmetrical(self.conntext.average_speed, [2,2,2])
    #         and self.conntext.vectorRegionCompare_symmetrical(self.conntext.as_array(self.conntext._average_wrench_gripper.force), self.max_force_error)
    #         and self.conntext.current_pose.transform.translation.z > self.conntext.restart_height):
    #         self.completion_confidence = self.completion_confidence + 1/self.rate_selected
    #         rospy.loginfo_throttle(1, Fore.RED + "Static. Restarting confidence: " + str( np.round(self.completion_confidence, 2) ) + " out of 1." +Style.RESET_ALL)
    #         #if((rospy.Time.now()-marked_time).to_sec() > .50): #if we've satisfied this condition for 1 second
    #         if(self.completion_confidence > 1):
    #                 #Restart Search
    #                 rospy.loginfo_throttle(1.0, "Restarting test!")
    #                 self.next_trigger, self.switch_state = self.post_action(RESTART_TEST_TRIGGER)
    #     else:
    #         self.completion_confidence = np.max( np.array([self.completion_confidence * 90/self.rate_selected, .01]))
    #         if(self.conntext.current_pose.transform.translation.z > self.conntext.restart_height):
    #             rospy.loginfo_throttle(1, Fore.RED + "That's high enough! Let robot stop and come to zero force." +Style.RESET_ALL)

    def all_states_calc(self):
        #All once-per-loop functions
        self.conntext.current_pose = self.conntext.get_current_pos()
        self.curr_time = rospy.get_rostime() - self.start_time
        self.curr_time_numpy = np.double(self.curr_time.to_sec())
        self.conntext.update_avg_speed()
        self.conntext.update_average_wrench()

    def checkForceCap(self):
        if(not self.conntext.force_cap_check()):
            self.next_trigger, self.switch_state = self.post_action(SAFETY_RETRACTION_TRIGGER)
            rospy.logerr("Force/torque unsafe; pausing application.")


class AssemblyStep:
    '''
    The default AssemblyStep provides a helpful structure to impliment discrete tasks in AssemblyBlocks. The default functionality below moves the TCP in a specified direction and ends when a rigid obstacle halts its motion. In general, the pattern goes thus:

    ::init:: runs when the Step is created, normally right before the first loop of its associated Step. The parameters entered in the AlgorithmBlocks.steps dictionary will be sent to the init function.

    ::execute:: runs each time the AlgorithmBlocks instance runs its loop. The continuous behavior of the robot should be defined here.

    ::checkCompletion:: runs each loop cycle, being triggered by the AlgorithmBlocks loop like 'execute' is. It checks the exitConditions method (below) to evaluate conditions, and gains/loses completion_confidence. The confidence behavior makes decision-making much more consistent, largely eliminating trouble from sensor noise, transient forces, and other disruptions. It returns a Boolean value; True should indicate that the exit conditions for the Step have been satisfied consistently and reliably. This triggers AlgorithmBlocks to run the Exit method. See below.

    ::exitConditions:: is the boolean "check" which checkCompletion uses to build/lose confidence that it is finished. Gives an instantaneous evaluation of conditions. Prone to noise due to sensor/control/transient messiness.

    ::onExit:: is run by the AlgorithmBlocks execution loop right before this Step object is Deleted. It should output the switch_state boolean (normally True since the step is done) and the trigger for the next step (normally STEP_COMPLETE_TRIGGER which simply moves us to the next Step in sequence, as dictated by the AlgorithmBlocks state machine). Any other end-of-step actions, like saving information to the AlgorithmBlocks object for later steps, can be done here.

    '''
    # from conntact.assembly_algorithm_blocks import AlgorithmBlocks

    def __init__(self, algorithmBlocks:(AlgorithmBlocks)) -> None:
        #set up the parameters for this step
        self.completion_confidence = 0.0
        self.seeking_force = [0,0,0]
        self.comply_axes = [1,1,1]
        # self.desiredOrientation = trfm.quaternion_from_euler(0,0,-90)
        self.desiredOrientation = trfm.quaternion_from_euler(0,0,0)
        self.done = False

        #Set up exit condition sensitivity
        self.exitPeriod = .5       #Seconds to stay within bounds
        self.exitThreshold = .99    #Percentage of time for the last period
        self.holdStartTime = 0;

        #Pass in a reference to the AlgorithmBlocks parent class; this reduces data copying in memory
        self.assembly:AlgorithmBlocks = algorithmBlocks
        self.conntext = self.assembly.conntext
        
    def execute(self):
        '''Executed once per loop while this State is active. By default, just runs UpdateCommands to keep the compliance motion profile running.
        '''
        self.updateCommands()

    def updateCommands(self):
        '''Updates the commanded position and wrench. These are published in the AlgorithmBlocks main loop.
        '''
        #Command wrench
        self.assembly.wrench_vec  = self.conntext.get_command_wrench(self.seeking_force)
        #Command pose
        self.assembly.pose_vec = self.conntext.arbitrary_axis_comply(self.comply_axes)


    def checkCompletion(self):
        """Check if the step is complete. Default behavior is to check the exit conditions and gain/lose confidence between 0 and 1. ExitConditions returning True adds a step toward 1; False steps down toward 0. Once confidence is above exitThreshold, a timer begins for duration exitPeriod.
        """

        if(self.exitConditions()):
            if(self.completion_confidence < 1):
                self.completion_confidence += 1/(self.assembly.rate_selected)

            if(self.completion_confidence > self.exitThreshold):
                if(self.holdStartTime == 0):
                    #Start counting down to completion as long as we don't drop below threshold again:
                    self.holdStartTime = rospy.get_time()

                elif(self.holdStartTime < rospy.get_time() - self.exitPeriod ):
                    #it's been long enough, exit loop
                    return True 
            else:
                # Confidence has dropped below the threshold, cancel the countdown.
                self.holdStartTime = 0
        else:
            #Exit conditions not true
            if(self.completion_confidence>0.0):
                self.completion_confidence -= 1/(self.assembly.rate_selected)

        return False

    def exitConditions(self)->bool:
        return self.noForce() 

    def static(self)->bool:
        return self.conntext.checkIfStatic()

    def collision(self)->bool:
        return self.conntext.checkIfColliding(np.array(self.seeking_force))

    def noForce(self)->bool:
        '''Checks the current forces against an expected force of zero, helpfully telling us if the robot is in free motion
        :return: (bool) whether the force is fairly close to zero.
        '''
        return self.conntext.checkIfColliding(np.zeros(3))

    def onExit(self):
        """Executed once, when the change-state trigger is registered.
        """
        return STEP_COMPLETE_TRIGGER, True


