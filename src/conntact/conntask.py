# Copyright 2021 Southwest Research Institute
# Licensed under the Apache License, Version 2.0

import string
import sys
from builtins import staticmethod

import numpy as np
# import rospy
import tf.transformations as trfm
from colorama import Back, Fore, Style, init as colorama_init

from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion, Transform,
                               TransformStamped, Vector3, Wrench,
                               WrenchStamped)
from transitions import Machine
from conntact.conntext import Conntext
import conntact.assembly_utils as utils

# State and Trigger names
# It is best to use tags like these for the string-only names of
# States and Transitions in your state machine to prevent spelling errors.

# Required state for Conntact's default features to work:
START_STATE = 'state_start'
EXIT_STATE = 'state_exit'
SAFETY_RETRACT_STATE = 'state_safety_retraction'

# Required trigger names:
SAFETY_RETRACTION_TRIGGER = 'retract to safety'
STEP_COMPLETE_TRIGGER = 'next step'
RUN_LOOP_TRIGGER = 'run looped code'


class ConnTask(Machine):
    """A parent class which conntains a state machine which creates, processes, and destroys ConnSteps in concert
    with state transitions. When you create a child class to implement Conntask (hereafter referred to as your Task),
    that class needs to provide the following to ConnTask: :param conntext: (Conntext) Reference to the Conntext
    object managing the environment. :param interface: (ConntactInterface) Reference to the interface object managing
    hardware/software connections. :param states: (List of str) state names. States in the state machine will be
    created based on these. :param transitions: (List of dictionaries) Format example below. List the name, source,
    and destination of each valid transition between states. These transitions are passed by name from ConnSteps back
    to Conntask to move to a new state. :param connfig_name: (str) name of the YAML file where configuration data is
    stored. All uses of your Task-specific params will be in your Task file, but Conntask will load them for you if
    asked. It looks within (your workspace)/src/Conntact/config for a .yaml file with the given name.

    # The following is an example of how to set up a state machine. Do this in your Task definition.
    #Declare the official states list here. These will be passed into the machine. The minimum required states are shown :
    states = [
        START_STATE
        EXIT_STATE,
        SAFETY_RETRACT_STATE
    ]

    #Define the valid transitions from/to each state. Here's where you define the topology of the state machine. The
    Machine executes the first transition in this list which matches BOTH the trigger AND the CURRENT state. If no
    other trigger is set at "self.next_trigger", Conntact will automatically fill in "RUN_LOOP_TRIGGER" which runs
    the Execute method of the current Step object.

    transitions = [
        {'trigger':SAFETY_RETRACTION_TRIGGER , 'source':'*'                 , 'dest':SAFETY_RETRACT_STATE
        unless':'is_already_retracting'},
        {'trigger':RESTART_TEST_TRIGGER      , 'source':SAFETY_RETRACT_STATE, 'dest':START_STATE   },
        {'trigger':RUN_LOOP_TRIGGER      , 'source':'*', 'dest':None, 'after': 'run_step_actions'}
    ]
    #Store a reference to the ConnStep class in use by each State:
    self.step_list:dict = {
        COMPLETION_STATE:     (ExitStep, [])
    }
    """

    def __init__(self, conntext, states, transitions, target_frame_name, connfig_name=None):
        self.conntext = conntext
        self.rate_selected = conntext.rate
        self.interface = self.conntext.interface
        self.conntext.set_target_frame_name(target_frame_name)

        self.connfig = self.interface.load_yaml_file(connfig_name)
        self.start_time = self.interface.get_unified_time()

        # Configuration variables, to be moved to a yaml file later:
        self.pose_command_vector = None
        self.wrench_command_vector = self.conntext.get_command_wrench([0, 0, 0])

        self.next_trigger = ''  # Empty to start. If no transition is assigned by the ConnStep
        # execution, Conntact fills in the default "RUN_LOOP_TRIGGER" which causes no transition
        # and simply executes the step.execute() method
        self.switch_state = False  # Triggers a state transition while preventing accidental repetitions

        self.current_step: ConnStep = None

        # Set up Colorama for colorful terminal outputs on all platforms
        colorama_init(autoreset=True)
        Machine.__init__(self, states=states, transitions=transitions, initial=START_STATE)

    def print(self, string: str):
        self.interface.send_info(Fore.LIGHTBLUE_EX + string + Style.RESET_ALL)

    def is_already_retracting(self):
        """This method makes use of the Machine's built-in
        "unless" feature. The named method is called and the boolean result
        permits or inhibits the transition. If the transition to retraction
        state is skipped, the default RUN_LOOP_TRIGGER is used.
        """
        return self.is_state_safety_retraction()

    def log_state_transition(self):
        self.interface.send_info(Fore.BLACK + Back.WHITE + "State transition to " +
                                 str(self.state) + " at time = " + str(
            self.interface.get_unified_time()) + Style.RESET_ALL)

    def send_commands(self):
        self.conntext.publish_pose(self.current_step.current_move)
        self.conntext.publish_wrench(self.current_step.wrench)

    # def set_command_wrench(self, force_vec, torque_vec = [0,0,0]):
    #     """
    #     For backward-compatability: set up the Step's move_policy based on the
    #     pose commands sent from it.
    #     """
    #     # self.wrench_command_vector = self.conntext.get_command_wrench(force_vec)
    #     #For backwards-compatibility: Take the input force command and send it to the move_policy
    #     self.current_step.force = force_vec
    #     self.current_step.torque = torque_vec

    # def set_command_pose(self, pose_vec):
    #     """
    #     For backward-compatability: set up the Step's move_policy based on the
    #     pose commands sent from it.
    #     """
    #     if self.current_step.move_policy.move_mode is None:
    #         #Categorize obsolete compliance axes vector into a movemode:
    #         lockedAxes = 0
    #         for i in pose_vec:
    #             if i==0:
    #                 lockedAxes +=1
    #         if lockedAxes == 0:
    #             self.current_step.move_policy = utils.MovePolicy("free")
    #         if lockedAxes == 1:
    #             #Permit motion along the given line:
    #             self.current_step.move_policy = utils.MovePolicy("line", vector=pose_vec)
    #         if lockedAxes == 2:
    #             #Get a plane normal perpendicular to the two motion axes:
    #             self.current_step.move_policy = utils.MovePolicy("plane", vector=np.array([1,1,1])-np.array(pose_vec))
    #         if lockedAxes == 3:
    #             self.current_step.move_policy = utils.MovePolicy("set")
    #         self.interface.send_info("Initialized new move policy.
    #         Configuration: {}".format(self.current_step.move_policy.info()))
    #     # # finally set the command:
    #     # self.pose_command_vector = self.current_step.current_move(self.conntext.current_pose.transform.translation)

    def run_step_actions(self):
        """Runs the ConnStep class associated with this state if one exists.
        It will fall back on a method with name matching the state name if no
        Step is listed.
        """
        state_name = str(self.state)
        if (state_name in self.step_list):
            # This step has been realized as a Step class
            if (not self.current_step):
                # Set step to an instance of the referred ConnStep and pass in the parameters.
                self.current_step = self.step_list[state_name][0](self, *self.step_list[state_name][1])
                self.interface.send_info(
                    Fore.GREEN + "Created step object " + str(type(self.current_step)) + Style.RESET_ALL)
                if self.current_step.move_policy is None:
                    # Update the legacy move setup to a move_policy
                    self.current_step.create_policy_from_legacy()
            else:
                self.current_step.execute()
            if self.current_step.check_completion():
                self.next_trigger, self.switch_state = self.current_step.on_exit()
        else:
            # Fall back on calling a method matching the state name. This is still allowed but discouraged.
            # consider using if you want to tie a State with a one-time action.
            method_name = "self." + state_name[state_name.find('state_') + 6:] + '()'
            try:
                self.interface.send_info(2, Fore.WHITE + "In state " + state_name + Style.RESET_ALL +
                                         ", now executing " + method_name)
                exec(method_name)
            except (NameError, AttributeError):
                self.interface.send_info(2, "State name " + method_name + " does not match 'state_'+"
                                                                          "(state loop method name) in algorithm!")
                pass
            except:
                print("Unexpected error when trying to locate state loop name:", sys.exc_info()[0])
                raise

    def algorithm_execute(self):
        """Main execution loop. A True exit state will cause the buffered Trigger "self.next_trigger" to be run, changing the state. If using a state realized as an ConnStep class, we delete the old step here. Also executes the once-per-cycle non-step commands needed for continuous safe operation.
        """
        while self.state != EXIT_STATE:
            # Main program loop.
            # Refresh values, run process trigger (either loop-back to perform state actions or transition to
            # new state), then output controller commands and wait for next loop time.

            if (self.switch_state):  # If the command to change states has come in:
                self.switch_state = False
                if (self.current_step):
                    # If a Step class has been defined for the current State, we delete it to be tidy.
                    del self.current_step
                    self.current_step = None
                self.trigger(self.next_trigger)

            else:
                # If we're not switching states, we use RUN_LOOP_TRIGGER to execute this state's loop code.
                self.next_trigger = RUN_LOOP_TRIGGER
                # Update state information (pose, sensor wrench):
                self.conntext.update()
                # Execute the trigger chosen. If not transitioning states, should just run state.execute, normally just updating the motion commands.
                self.trigger(self.next_trigger)
                self.checkForceCap()
                self.send_commands()

            # Publish robot motion commands only once per loop, right at the end of the loop:
            self.interface.sleep_until_next_loop()

    def checkForceCap(self):
        if (not self.conntext.force_cap_check()):
            self.next_trigger = SAFETY_RETRACTION_TRIGGER
            self.interface.send_error("Force/torque unsafe; pausing application.")


class ConnStep:
    '''
    The default ConnStep provides a helpful structure to connduct discrete tasks in a ConnTask.
    The default functionality below moves the TCP in a specified direction and ends when a rigid obstacle halts its
    motion. In general, the pattern goes thus:
    ::init:: runs when the Step is created, normally right before the first loop of its associated Step. The parameters
    entered in the ConnTask.steps dictionary will be sent to the init function.
    ::execute:: runs each time the ConnTask instance runs its loop. The continuous behavior of the robot should be
    defined here.
    ::check_completion:: runs each loop cycle, being triggered by the ConnTask loop like 'execute' is. It checks the
    exit_conditions method (below) to evaluate conditions, and gains/loses completion_confidence. The confidence
    behavior makes decision-making much more consistent, largely eliminating trouble from sensor noise, transient
    forces, and other disruptions. It returns a Boolean value; True should indicate that the exit conditions for the
    Step have been satisfied consistently and reliably. This triggers ConnTask to run the Exit method. See below.
    ::exit_conditions:: is the boolean "check" which check_completion uses to build/lose confidence that it is finished.
     Gives an instantaneous evaluation of conditions. Prone to noise due to sensor/control/transient messiness.
    ::on_exit:: is run by the ConnTask execution loop right before this Step object is Deleted. It should output the
    switch_state boolean (normally True since the step is done) and the trigger for the next step
    (normally STEP_COMPLETE_TRIGGER which simply moves us to the next Step in sequence, as dictated by the
    ConnTask state machine). Any other end-of-step actions, like saving information to the ConnTask object for
    later steps, can be done here.
    '''

    def __init__(self, connTask: (ConnTask)) -> None:
        # set up the parameters for this step
        if not hasattr(self, "_move_policy"):
            # Make sure an empty move_policy is present; don't overwrite though
            self._move_policy = None
        self.completion_confidence = 0.0
        self.desiredOrientation = trfm.quaternion_from_euler(0, 0, 0)

        # Set up exit condition sensitivity
        self.exitPeriod = .5  # Seconds to stay within bounds
        self.exitThreshold = .90  # Percentage of time for the last period
        self.holdStartTime = 0

        # Pass in a reference to the ConnTask parent object:
        self.task: ConnTask = connTask
        self.conntext = self.task.conntext

    @property
    def move_policy(self):
        return self._move_policy

    @move_policy.setter
    def move_policy(self, value):
        """You can pass in a MovePolicy to attach to the ConnStep."""
        self._move_policy = value

    @property
    def wrench(self):
        return self.move_policy.wrench

    @property
    def current_move(self):
        return self.move_policy.current_move(self.conntext.current_pose.transform.translation)

    def create_policy_from_legacy(self):
        """
        Categorize obsolete compliance axes vector into a move_mode.
        """
        if hasattr(self, "comply_axes"):
            lockedAxes = 0
            for i in self.comply_axes:
                if i == 0:
                    lockedAxes += 1
            if lockedAxes == 0:
                self.create_move_policy(move_mode="free")
            if lockedAxes == 1:
                # Permit motion along the given line:
                self.create_move_policy(move_mode="line",
                                        vector=self.comply_axes)
            if lockedAxes == 2:
                # Get a plane normal perpendicular to the two motion axes:
                self.create_move_policy(move_mode="plane",
                                        vector=np.array([1, 1, 1])
                                               - np.array(self.comply_axes))
            if lockedAxes == 3:
                self.create_move_policy(move_mode="set")
        else:
            self.create_move_policy(move_mode="free")
        if hasattr(self, "seeking_force"):
            self.move_policy.force = self.seeking_force
        self.task.interface.send_info("Initialized new move policy."
                                      "Configuration: {}".format(self.move_policy.info()))

    def create_move_policy(self,
                           move_mode: str = None,
                           vector=None, origin=None,
                           orientation=[0, 0, 0],
                           force=[0, 0, 0], torque=[0, 0, 0]):
        """
        Create a move_policy from arguments:
        MoveMode:
        :param MoveMode:  (np.array(3)) Select the movement mode from the list,
            and a policy object will be created to maintian that policy.
            "Set": The robot will attempt to move to the exact coordinates passed as Origin.
            "Line": The robot will move freely along a vector in target space. Vector's origin
                robot TCP position when the MoveMode is selected; to move the start point, pass
                in optional command "start_point".
            "Plane": As "line" MoveMode, except the robot will hold to the plane normal to the
                vector and passing through the origin point.
            "Free": The robot moves freely in all directions.
        :param vector: (np.array(3)) In Line or Plane MoveMode, the vector onto/about which the robot's
            current position is projected to generate the commanded position.
        :param origin: (np.array(3)) In the Set MoveMoveMode, the point the robot will try to attain. In Line
            or Plane MoveMode, the origin point of the vector. When you set/change the MoveMode, this point
            updates to the robot TCP's current position by default, unless you pass a value in.
        :param force_cmd:  (list of floats) Force to apply at the TCP. Moves the robot through space
            according to MoveMode. An equal and opposite force from the physical sensor will stop this motion.
        :param torque_cmd:  (list of floats) XYZ torques to apply relative to target.
        """
        self._move_policy = utils.MovePolicy(move_mode,
                                             vector,
                                             origin,
                                             orientation,
                                             force,
                                             torque)

    def reset_move_policy(self):
        self._move_policy = utils.MovePolicy()

    def execute(self):
        '''
        Executed once per loop while this State is active. By default, just runs update_commands to keep the compliance
        motion profile running.
        '''
        # self.update_commands() # TODO: remove this completely.
        pass

    def update_commands(self):
        '''
        Updates the commanded position and wrench. These are published in the ConnTask main loop.
        '''
        # Command wrench
        self.task.set_command_wrench(self.seeking_force)
        # Command pose
        self.task.set_command_pose(self.comply_axes)

    def check_completion(self):
        """Check if the step is complete. Default behavior is to check the exit conditions and gain/lose confidence
        between 0 and 1. exit_conditions returning True adds a step toward 1; False steps down toward 0.
        Once confidence is above exitThreshold, a timer begins for duration exitPeriod.
        """
        if self.exit_conditions():
            if self.completion_confidence < 1:
                if self.completion_confidence < self.exitThreshold:
                    self.completion_confidence = self.exitThreshold
                else:
                    self.completion_confidence += 1 / (self.task.rate_selected)
        else:
            # Exit conditions not true
            if self.completion_confidence > 0:
                self.completion_confidence -= 10 * (self.exitThreshold) / (self.task.rate_selected)

        # Now decide if we're reasonably confident after the hold_time interval
        if self.completion_confidence > self.exitThreshold:
            if self.holdStartTime == 0:
                # Start counting down to completion as long as we don't drop below threshold again:
                self.holdStartTime = self.task.interface.get_unified_time(float=True)
            elif self.holdStartTime < self.task.interface.get_unified_time(float=True) - self.exitPeriod:
                # it's been long enough, exit loop
                return True
        else:
            # Confidence has dropped below the threshold, cancel the countdown.
            self.holdStartTime = 0

        return False

    def exit_conditions(self) -> bool:
        return self.no_force()

    def is_static(self) -> bool:
        return self.conntext.checkIfStatic()

    def in_collision(self) -> bool:
        return self.conntext.checkIfColliding(np.array(self.seeking_force))

    def no_force(self) -> bool:
        '''Checks the current forces against an expected force of zero, helpfully telling us if the robot is in free motion
        :return: (bool) whether the force is fairly close to zero.
        '''
        return self.conntext.checkIfColliding(np.zeros(3))

    def on_exit(self):
        """Executed once, when the change-state trigger is registered.
        """
        return STEP_COMPLETE_TRIGGER, True
