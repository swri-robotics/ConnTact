
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

from peg_in_hole_demo.assembly_tools import AssemblyTools

from transitions import Machine

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
RUN_LOOP_TRIGGER           = 'run looped code'



class AlgorithmBlocks(AssemblyTools):

    def __init__(self, ROS_rate, start_time):
        # self._wrench_pub    = rospy.Publisher('/cartesian_compliance_controller/target_wrench', WrenchStamped, queue_size=10)
        # self._pose_pub      = rospy.Publisher('cartesian_compliance_controller/target_frame', PoseStamped , queue_size=2)
        # self._target_pub    = rospy.Publisher('target_hole_position', PoseStamped, queue_size=2, latch=True)
        # self._ft_sensor_sub = rospy.Subscriber("/cartesian_compliance_controller/ft_sensor_wrench/", WrenchStamped, self.callback_update_wrench, queue_size=2)

        #Configuration variables, to be moved to a yaml file later:
        self.speed_static = [1/1000,1/1000,1/1000]          #Speed at which the system considers itself stopped. Rel. to target hole.

        force_dangerous = [45,45,55]                        #Force value which kills the program. Rel. to gripper.
        force_transverse_dangerous = np.array([30,30,30])   #Force value transverse to the line from the TCP to the force sensor which kills the program. Rel. to gripper.
        force_warning = [25,25,25]                          #Force value which pauses the program. Rel. to gripper.
        force_transverse_warning = np.array([20,20,20])     #torque value transverse to the line from the TCP to the force sensor which kills the program. Rel. to gripper.
        self.max_force_error = [3.5, 3.5, 5]                #Allowable error force with no actual loads on the gripper.
        self.cap_check_forces = force_dangerous, force_transverse_dangerous, force_warning, force_transverse_warning 


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
            {'trigger':FIND_HOLE_TRIGGER         , 'source':APPROACH_STATE      , 'dest':FIND_HOLE_STATE        },
            {'trigger':INSERT_PEG_TRIGGER        , 'source':FIND_HOLE_STATE     , 'dest':INSERTING_PEG_STATE    },
            {'trigger':ASSEMBLY_COMPLETED_TRIGGER, 'source':INSERTING_PEG_STATE , 'dest':COMPLETION_STATE       },
            {'trigger':SAFETY_RETRACTION_TRIGGER , 'source':'*'                 , 'dest':SAFETY_RETRACT_STATE, 'unless':'is_already_retracting' },
            {'trigger':RESTART_TEST_TRIGGER      , 'source':SAFETY_RETRACT_STATE, 'dest':CHECK_FEEDBACK_STATE   },
            {'trigger':RUN_LOOP_TRIGGER      , 'source':'*', 'dest':None, 'after': 'run_loop'}

        ]
       
        Machine.__init__(self, states=states, transitions=transitions, initial=IDLE_STATE)
        
        
        AssemblyTools.__init__(self, ROS_rate, start_time)
        # Set up Colorama for colorful terminal outputs on all platforms
        init(autoreset=True)
        # temporary selector for this algorithm's TCP; easily switch from tip to corner-centrered search 
        self.tcp_selected = 'tip'



    def is_already_retracting(self):
        return self.is_state_safety_retraction()
    def on_enter_state_check_load_cell_feedback(self):
        # self.select_tool('corner')
        self.reset_on_state_enter()
        self.select_tool(self.tcp_selected)
        self._log_state_transition()
    def on_enter_state_finding_surface(self):
        # self.select_tool('corner')
        self.reset_on_state_enter()
        self.select_tool(self.tcp_selected)
        self._log_state_transition()
    def on_enter_state_finding_hole(self):
        # self.select_tool('corner')
        self.reset_on_state_enter()
        self.select_tool(self.tcp_selected)
        self._log_state_transition()
    def on_enter_state_inserting_along_axis(self):
        self.reset_on_state_enter()
        self._log_state_transition()
    def on_enter_state_completed_insertion(self):
        self.reset_on_state_enter()
        self._log_state_transition()
    def on_enter_state_retracing_to_safety(self):
        self.reset_on_state_enter()
        self._log_state_transition()


    def _log_state_transition(self):
        rospy.loginfo(Fore.BLACK + Back.WHITE +"State transition to " + str(self.state) + " at time = " + str(rospy.get_rostime()) + Style.RESET_ALL )
    def reset_on_state_enter(self):
        self.collision_confidence = 0

    def update_commands(self):
        rospy.logerr_once("Preparing to publish pose: " + str(self.pose_vec) + " and wrench: " + str(self.wrench_vec))
        self.publish_pose(self.pose_vec)
        self.publish_wrench(self.wrench_vec)
        

    def run_loop(self):
        state_name=str(self.state)
        if("state_") in state_name:
            method_name = "self."+state_name[state_name.find('state_')+6:]+'()'
        else:
            rospy.logerr("Invalid state name! Terminating.")
            quit()
        try:
            rospy.loginfo_throttle(2, Fore.WHITE + "In state "+state_name + Style.RESET_ALL)
            exec(method_name)
        except (NameError, AttributeError):
            rospy.logerr_throttle(2, "State name " + method_name + " does not match 'state_'+(state loop method name) in algorithm!")
            pass
        except:
            print("Unexpected error when trying to locate state loop name:", sys.exc_info()[0])
            raise


    def check_load_cell_feedback(self):
        # self.switch_state = False
        #Take an average of static sensor reading to check that it's stable.

        if (self.curr_time_numpy > 2):
            self._bias_wrench = self._average_wrench_gripper
            rospy.logerr("Measured bias wrench: " + str(self._bias_wrench))

            self.force_cap_check(*self.cap_check_forces)
            if(not self.highForceWarning):
                rospy.logerr("Starting linear search.")
                self.next_trigger, self.switch_state = self.post_action(APPROACH_SURFACE_TRIGGER) 
                
            else:
                rospy.logerr("Starting wrench is dangerously high. Suspending. Try restarting robot if values seem wrong.")
                self.next_trigger, self.switch_state = self.post_action(SAFETY_RETRACTION_TRIGGER) 


    def finding_surface(self):
        #seek in Z direction until we stop moving for about 1 second. 
        # Also requires "seeking_force" to be compensated pretty exactly by a static surface.
        #Take an average of static sensor reading to check that it's stable.

        seeking_force = 5
        self.wrench_vec  = self.get_command_wrench([0,0,seeking_force])
        self.pose_vec = self.linear_search_position([0,0,0]) #doesn't orbit, just drops straight downward

        if(not self.force_cap_check(*self.cap_check_forces)):
            self.next_trigger, self.switch_state = self.post_action(SAFETY_RETRACTION_TRIGGER) 
            rospy.logerr("Force/torque unsafe; pausing application.")
        elif( self.vectorRegionCompare_symmetrical(self.average_speed, self.speed_static) 
            and self.vectorRegionCompare(self.as_array(self._average_wrench_world.force), [10,10,seeking_force*1.5], [-10,-10,seeking_force*.75])):
            self.collision_confidence = self.collision_confidence + 1/self._rate_selected
            rospy.logerr_throttle(1, "Monitoring for flat surface, confidence = " + str(self.collision_confidence))
            #if((rospy.Time.now()-marked_time).to_sec() > .50): #if we've satisfied this condition for 1 second
            if(self.collision_confidence > .90):
                #Stopped moving vertically and in contact with something that counters push force
                rospy.logerr("Flat surface detected! Moving to spiral search!")
                #Measure flat surface height:
                self.surface_height = self.current_pose.transform.translation.z
                self.next_trigger, self.switch_state = self.post_action(FIND_HOLE_TRIGGER) 
                self.collision_confidence = 0.01
        else:
            self.collision_confidence = np.max( np.array([self.collision_confidence * 95/self._rate_selected, .001]))

        
            # self.activeTCP = origTCP

    def finding_hole(self):
        #Spiral until we descend 1/3 the specified hole depth (provisional fraction)
        #This triggers the hole position estimate to be updated to limit crazy
        #forces and oscillations. Also reduces spiral size.
       

        seeking_force = 7.0
        self.wrench_vec  = self.get_command_wrench([0,0,seeking_force])
        self.pose_vec = self.spiral_search_basic_compliance_control()

        if(not self.force_cap_check(*self.cap_check_forces)):
            self.next_trigger, self.switch_state = self.post_action(SAFETY_RETRACTION_TRIGGER) 
            rospy.logerr("Force/torque unsafe; pausing application.")
        elif( self.current_pose.transform.translation.z <= self.surface_height - .0004):
            #If we've descended at least 5mm below the flat surface detected, consider it a hole.
            self.collision_confidence = self.collision_confidence + 1/self._rate_selected
            rospy.logerr_throttle(1, "Monitoring for hole location, confidence = " + str(self.collision_confidence))
            if(self.collision_confidence > .90):
                    #Descended from surface detection point. Updating hole location estimate.
                    self.x_pos_offset = self.current_pose.transform.translation.x
                    self.y_pos_offset = self.current_pose.transform.translation.y
                    self._amp_limit_cp = 2 * np.pi * 4 #limits to 3 spirals outward before returning to center.
                    #TODO - Make these runtime changes pass as parameters to the "spiral_search_basic_compliance_control" function
                    rospy.logerr_throttle(1.0, "Hole found, peg inserting...")
                    self.next_trigger, self.switch_state = self.post_action(INSERT_PEG_TRIGGER) 
        else:
            self.collision_confidence = np.max( np.array([self.collision_confidence * 95/self._rate_selected, .01]))
            if(self.current_pose.transform.translation.z >= self.surface_height - self.hole_depth):
                rospy.loginfo_throttle(1, Fore.YELLOW + "Height is still " + str(self.current_pose.transform.translation.z) 
                    + " whereas we should drop down to " + str(self.surface_height - self.hole_depth) + Style.RESET_ALL )

            

    def inserting_along_axis(self):
        #Continue spiraling downward. Outward normal force is used to verify that the peg can't move
        #horizontally. We keep going until vertical speed is very near to zero.


        seeking_force = 5.0
        self.wrench_vec  = self.get_command_wrench([0,0,seeking_force])
        self.pose_vec = self.full_compliance_position()

        if(not self.force_cap_check(*self.cap_check_forces)):
            self.next_trigger, self.switch_state = self.post_action(SAFETY_RETRACTION_TRIGGER) 
            rospy.logerr("Force/torque unsafe; pausing application.")
        elif( self.vectorRegionCompare_symmetrical(self.average_speed, self.speed_static) 
            #and not self.vectorRegionCompare(self.as_array(self._average_wrench_world.force), [6,6,80], [-6,-6,-80])
            and self.vectorRegionCompare(self.as_array(self._average_wrench_world.force), [1.5,1.5,seeking_force*1.5], [-1.5,-1.5,seeking_force*-.75])
            and self.current_pose.transform.translation.z <= self.surface_height - self.hole_depth):
            self.collision_confidence = self.collision_confidence + 1/self._rate_selected
            rospy.logerr_throttle(1, "Monitoring for peg insertion, confidence = " + str(self.collision_confidence))
            #if((rospy.Time.now()-marked_time).to_sec() > .50): #if we've satisfied this condition for 1 second
            if(self.collision_confidence > .90):
                    #Stopped moving vertically and in contact with something that counters push force
                    self.next_trigger, self.switch_state = self.post_action(ASSEMBLY_COMPLETED_TRIGGER) 
        else:
            #
            self.collision_confidence = np.max( np.array([self.collision_confidence * 95/self._rate_selected, .01]))
            if(self.current_pose.transform.translation.z >= self.surface_height - self.hole_depth):
                rospy.loginfo_throttle(1, Fore.YELLOW + "Height is still " + str(self.current_pose.transform.translation.z) 
                    + " whereas we should drop down to " + str(self.surface_height - self.hole_depth) + Style.RESET_ALL)

            

    def completed_insertion(self):
        #Inserted properly.
     
        rospy.loginfo_throttle(1, Fore.RED + "Hole found, peg inserted! Done!" +Style.RESET_ALL)
        if(self.current_pose.transform.translation.z > self.restart_height+.02):
            #High enough, won't pull itself upward.
            seeking_force = -2.5
            rospy.loginfo_once(Back.GREEN + Fore.WHITE + Style.BRIGHT + "Completed Task!" + Style.RESET_ALL)
            quit()
        else:
            #pull upward gently to move out of trouble hopefully.
            seeking_force = -20
        self.force_cap_check(*self.cap_check_forces)
        self.wrench_vec  = self.get_command_wrench([0,0,seeking_force])
        self.pose_vec = self.full_compliance_position()

            

    def safety_retraction(self):
        #Safety passivation; chill and pull out. Actually restarts itself if everything's chill enough.


        if(self.current_pose.transform.translation.z > self.restart_height+.05):
            #High enough, won't pull itself upward.
            seeking_force = -3.5
        else:
            #pull upward gently to move out of trouble.
            seeking_force = -10
        self.wrench_vec  = self.get_command_wrench([0,0,seeking_force])
        self.pose_vec = self.full_compliance_position()

        rospy.loginfo_throttle(1, Fore.RED + "Task suspended for safety. Freewheeling until low forces and height reset above " + str(self.restart_height) + ': ' + str(self.current_pose.transform.translation.z) + Style.RESET_ALL)
        if( self.vectorRegionCompare_symmetrical(self.average_speed, [2,2,2]) 
            and self.vectorRegionCompare_symmetrical(self.as_array(self._average_wrench_gripper.force), self.max_force_error)
            and self.current_pose.transform.translation.z > self.restart_height):
            self.collision_confidence = self.collision_confidence + 1/self._rate_selected
            rospy.loginfo_throttle(1, Fore.RED + "Static. Restarting confidence: " + str( np.round(self.collision_confidence, 2) ) + " out of 1." +Style.RESET_ALL)
            #if((rospy.Time.now()-marked_time).to_sec() > .50): #if we've satisfied this condition for 1 second
            if(self.collision_confidence > 1):
                    #Restart Search
                    rospy.loginfo_throttle(1.0, "Restarting test!")
                    self.next_trigger, self.switch_state = self.post_action(RESTART_TEST_TRIGGER) 
        else:
            self.collision_confidence = np.max( np.array([self.collision_confidence * 90/self._rate_selected, .01]))
            if(self.current_pose.transform.translation.z > self.restart_height):
                rospy.loginfo_throttle(1, Fore.RED + "That's high enough! Let robot stop and come to zero force." +Style.RESET_ALL)

            

    #All state callbacks need to calculate this in a while loop
    def all_states_calc(self):
        #All once-per-loop functions
        self.current_pose = self.get_current_pos()
        self.curr_time = rospy.get_rostime() - self._start_time
        self.curr_time_numpy = np.double(self.curr_time.to_sec())
        marked_state = 1; #returns to this state after a soft restart in state 99
        # self.wrench_vec  = self.get_command_wrench([0,0,-2])
        # self.pose_vec = self.full_compliance_position()
        self.update_avg_speed()
        self.update_average_wrench()
        # self._update_plots()
        rospy.loginfo_throttle(1, Fore.BLUE + "Average wrench in newtons  is force \n" + str(self._average_wrench_world.force)+ 
            " and torque \n" + str(self._average_wrench_world.torque) + Style.RESET_ALL)
        rospy.loginfo_throttle(1, Fore.CYAN + "\nAverage speed in mm/second is \n" + str(1000*self.average_speed) +Style.RESET_ALL)

        self.publish_plotted_values()

    def algorithm_execute(self):

        self.collision_confidence = 0
        
        self.next_trigger, self.switch_state = self.post_action(CHECK_FEEDBACK_TRIGGER)

        rospy.loginfo(Fore.BLACK + Back.GREEN + "Beginning search algorithm. "+Style.RESET_ALL)

        while not rospy.is_shutdown() and self.state != EXIT_STATE:
            # Main program loop. Refresh values, run process trigger (either loop-back to perform state actions or transition to new state), then output controller commands and wait for next loop time.
            self.all_states_calc()
            
            if(not self.switch_state):
                self.next_trigger = RUN_LOOP_TRIGGER
            else:
                self.switch_state = False
            self.trigger(self.next_trigger)

            # self.next_trigger, self.switch_state = self.post_action(RUN_LOOP_TRIGGER)

            self.update_commands()
            self._rate.sleep()
                    