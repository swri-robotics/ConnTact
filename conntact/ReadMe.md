# ConnTact

**A software framework to enable agile robotic assembly applications.**


Etymology:

**_Conn_** (intransitive verb)
 to conduct or direct the steering of (a vessel, such as a ship)

**_Tact_** (noun)
 Careful delicacy to achieve a purpose without causing harm

_Alternative etymology: Connect + Tactile_

## Overview

The ConnTact package provide an efficient framework for assembly algorithm development for compliant robots. It allows the user to define fully tactile methods for making tight-tolerance connections in a human-like way.

ConnTact includes an implementation of the [transitions](https://github.com/pytransitions/transitions) state machine package to define the steps and decisions critical to your algorithm. It also provides a suite of tools and examples to sense the environment based on force feedback - detecting collision, hard surfaces, and position changes. Finally we define some basic motion profiles which can be used to probe the environment and align the tool to the workpiece.

![Framework diagram](resource/Conntact_Overview_Diagram_small.png)

The above diagram illustrates the topology of a Conntact implementation. The user's main program sets up the ConnTact environment by starting up persistent Conntext and ConntactInterface objects. These manage environment communication and interpretation. Then, the user program can run sequential ConnTasks in this prepared environment to carry out different manipulation tasks.

The ConnTask is the basic unit of ConnTact implementation: it's a user-created state machine describing an algorithmic solution to a problem. ConnTasks are easy to build and test in ConnTact as long as you can break down the human method into discrete motions and decisions. Here we show that the user has developed a screw-driving Task, a plug-insertion Task, and a gear-assembling Task.  

We show that the ConnTask has a few ConnStep objects inside. These are simple definitions of motion and end conditions which give executable meaning to the ConnTask state machine states.

Above the InsertScrew ConnTask we show some Connfig files, each for a different size of screw. Connfigs hold parameters to specify a Task for different sizes/shapes of workpieces of the same basic type. Most algorithms are expected to be easy to parameterize. Using Connfigs, numerical values are easily tweakable for different workpieces.

## Installation

Development of framework was done under Ubuntu Focal (20.04) using [ROS Noetic](http://wiki.ros.org/noetic).
  - If not previously installed, install [vcstools](http://wiki.ros.org/vcstool) and [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).
  - Initialize ROS workspace (e.g., `mkdir -p ~/ros_ws/src`, `cd ~/ros_ws/`, `catkin init`)
  - Clone the repository into the `src/` directory of a ROS workspace (e.g., `~/ros_ws/src`)
  - Install Python dependencies `pip install transitions modern_robotics`
  - Install ROS source dependencies:
    - `cd ~/ros_ws/src`
    - `vcs import < ConnTact/dependencies.rosinstall` 
     - We are running Conntact on a UR-10e; this command will install all dependencies needed for the UR implementation `Universal_Robots_ROS_Driver`, `fmauch_universal_robot`, `ur_msgs`). If not operating a UR, you can remove these.
  - Install ROS package dependencies: `rosdep install --rosdistro noetic --ignore-src --from-paths .`
  - Source ROS and build the workspace: `. /opt/ros/noetic/setup.bash`, `catkin build`

#### Universal Robot Users:
If you intend to run Conntact on a UR, it may be required to uninstall the ROS-default `ros-noetic-ur-client-library` - it uses old debs which lack major performance improvements. Even when running on a highly-performant computer, the RTDE buffer may overflow, giving `Pipeline Producer overflowed! <RTDE Pipeline>` errors. This causes major instability in robot performance. 

Fix this by building the more recent versions from source:

- `sudo apt remove ros-noetic-ur-client-library`
- `sudo apt install ccache`
- `git clone https://github.com/UniversalRobots/Universal_Robots_Client_Library.git`
- `catkin config -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_C_COMPILER_LAUNCHER=ccache`

## Structure

![Detailed block diagram](resource/Conntact_Detail_Diagram.png)

ConnTact is structured with 4 levels of functionality:

#### ConntactInterface

 The Interface defines the complete list of abstract functions which ConnTact uses to interact with a computer-robot system (hardware and software environment). As long as these functions are implemented as intended, ConnTact will run on any hardware and will command any robot. We provide working Interface examples for the UR10e running in both ROS and ROS 2 (coming soon).

#### Conntext

  The Conntext package acts as a wrapper which translates world-space data from the interface into the _task space_: the frame-of-reference of the task to be accomplished. This way, an algorithm is automatically reoriented and repositioned to each job, i.e. different hole positions and orientations for a screw-driving application. Conntext also provides useful data utilities, such as speed estimation and sensor data filtering.

  See Appendix: Interpolation for more info.


#### ConnTask

  To use ConnTact, the user creates a ConnTask implementation. Conntask is an easily-reconfigurable state machine which automatically associates an ConnStep behavior (see below) with each state and transition. Once you break a job down into sequential tasks, you can quickly build the functional skeleton of the algorithm.

#### ConnStep

  ConnTask implements tactile sensing in a pairing of *movement profile* and *end conditions*. A *movement profile* describes axes of force, compliance, or resisted motion to be executed by the robot end effector. By default, you define this profile in a MovePolicy (details below). The movement profile drives the robot through space and along surfaces. An *end condition* is a description of a force, torque, or motion signal which indicates that the robot has encountered a specific feature of interest which should end the motion profile and move the state machine. 
  The ConnStep class makes it extremely easy to first define a motion profile to move the robot through the environment, and then to define the end conditions which indicate that the motion has reached either its goal or an obstacle. Depending on the end condition detected, the ConnStep instructs ConnTask's state machine which specific next step to which to transition.

###### MovePolicy

Each ConnStep has a MovePolicy which defines its axes of set position and axes of continuous movement in response to forces.

For example, in the SpiralSearch example, the robot needs to find the real Z position of the workpiece surface before inserting the peg; therefor it needs to hold the expected X and Y position of the hole, and move along the Z axis until an obstacle stops it. X and Y are set, while Z moves continuously.

<details><summary>MovePolicy under-the-hood</summary>

We realize MovePolicy motion in Conntact by continuously updating CartesianComplianceController's (CCC's) `target_frame` topic, the pose to which the robot is trying to move. The robot will move toward this goal pose with a PD feedback system. With CCC, this setpoint-seeking behavior is additively combined with the disturbance-response behavior: At any time, forces applied to the robot will move it away from its setpoint or keep it from reaching it.

By updating the `target_frame` to the robot's current position, one can eliminate the setpoint-seeking behavior: Disturbance forces move the robot, and the robot does not seek to return to its undisturbed position. A `target_force` sent to the robot acts identically to an outside disturbance force, so in this fully compliant mode, the robot will constantly move in response to a force command until outside forces cancel it and stop the motion. 

Note that this is not the only way to achieve this continuous behavior with CCC. We choose to negate the setpoint-seeking behavior and move the robot by applying a force.

The inverse would work: Continually update the setpoint a measured distance from the robot's current position to drive motion by "leading" the robot. Indeed it would be possible to snap the setpoint beyond the expected target location and let the robot find an obstacle by running into it on its way to the setpoin.

We decided that this 1) can result in uncontrolled or variable robot speeds and 2) does not produce a controllable amount of contact force. The force-driven motion created by MovePolicy creates a reliable speed because CCC's inherent damping is always applied at the robot's update speed of 500 hz regardless of Conntact's cycle speed. Our method also can never produce more force (in the movement direction) than the command force given in the Policy.

</details>

The *MovePolicy* system allows you to update the setpoint *along an arbitrary axis/set of axes* to create continuous, force-sensitive motion where desired and to hold steady to a known position otherwise. 

The MovePolicy's Origin (shown as a black point) and Vector (shown in grey) are used to form a 3D vector at a 3D origin point. MovePolicy uses 4 MoveModes to describe the number of axes constrained, and constrains them to Origin-Vector accordingly:

![MoveMode description](resource/MoveModes.png)

This diagram shows how the robot's current position is projected onto the origin-vector combination by different MoveModes to find the command setpoint position which will be sent to CCC.  These values are defined in task space.

 - **Line** mode projects the robot's position onto the vector. This has the effect that the robot will only move along the vector.
 - **Plane** projects the robot's position onto the plane passing through Origin and normal to Vector, allowing 2D motion while holding the other axis static.
 - **Set** does not require Vector; the robot's command position is always set to Origin.
 - **Free** does not require Vector or Origin; it always updates the command position to the robot's current position, effectively negating the setpoint-seeking behavior and permitting continuous motion in any direction.

Note also that, for Set, Line, or Plane mode, if you don't pass Origin to the MoveMode constuctor, it will record the robot's current position, making it easy to continue a motion in a different direction.

###### Modifying/Overriding the MovePolicy

To define more complex motions, you can override your Step's inbuilt `current_move` property to change MovePolicy behavior (origin, vector, Mode, etc.) whenever the policy is accessed:

```
@property
def current_move(self):
    self.move_policy.origin = self.get_spiral_search_pose()
    return super().current_move
```

This override can be used to completely ignore MovePolicy if you prefer to write your own motion control code. See the source code to find the expected return values.

Alternately you can add the update code to your step's `execute()` method, convenient if your step is already using it for once-per-cycle tasks:

The SpiralToFindHole Step in the SpiralSearch example uses this approach to mathematically define a motion path for X and Y while allowing the tool to slide across a surface in Z, fully compliant and seeking an irregularity.

```
def execute(self):
    self.move_policy.origin = self.get_spiral_search_pose()
```

Note that MovePolicy also stores the Force, Torque and Orientation commands for the Step. It doesn't currently provide any help for creating continuous *rotational* compliance; if complex rotational behaviors are needed, either update the orientation command to the current orientation, or make your own compromise for now. We're very open to pull requests on this!

See `assembly_utils/MovePolicy` for more info.

## Development

We suggest the following workflow to take a task from a human and give it to a robot. We'll use the example of inserting a peg into a hole to illustrate each step. This is the task for which the included `SpiralSearch` example package was developed. Snippets of code from `SpiralSearch` are included in collapsed sections to illustrate how little user programming is needed.  

#### 1. Choose the frame-of-reference for your task.
  * The hole is vertical. We'll establish the Z axis to be vertical, concentric with the hole. All positions from here onward will be measured relative to the hole's position and orientation. We also establish the peg's Z axis along its axis pointing away from the robot flange. 

#### 2. Break the human task down into a sequence of tactile *steps* and *decisions*.
  * Roughly align the Z axes of the peg with the expected hole position by sending it to the task-space position (x, y, z) = (0,0,10). We specify that the robot begin to search for the surface of the assembly board from a safe height (10 cm) to prevent accidental collision. 
  * Move the peg downward (-z) until it hits the surface where the hole is bored.
  * Measure the exact surface height for later reference.
  * Slide the peg along the surface in an outward spiral until it catches in the hole and falls below the detected surface by 1 mm. If not detected, return to the center and try again.
  * Once the hole location is found, push the peg downward as far as possible, complying in all axes. The compliant robot will automatically align with the hole in orientation and position.
  * When the robot reaches a static obstacle that stops the peg's motion, the insertion is complete!
  * Retract the robot to reset the test.

#### 3. Define the state machine based on this plan.

  The pytransitions package makes state machine setup very straightforward, and Conntact doesn't need much more information to carry out its job.

  The SpiralSearch example follows the steps laid out in **2** very closely; however, there are a few items not discussed above which we have built into ConnTact.
  - There is one required transition for all ConnTask state machines, shown below. This is a _reflexive_ transition; that is, it leads from any state back to the same state. This permits every loop cycle to end by returning a trigger. This reflexive trigger also activates the current ConnStep's `execute` code through the `run_step_actions` callback, creating the motion behavior commanded. Don't leave this out!
    `{'trigger':RUN_LOOP_TRIGGER          , 'source':'*'                 , 'dest':None, 'after': 'run_step_actions'}`
  - ConnTact is currently equipped with a safety retraction feature. This feature protects the robot and workcell from damage from unexpected robot motion or runaway feedback oscillation, which is always a danger when running a robot with realtime control rather than planned paths. If forces rise above a specified level for a specified time, the state machine receives a `SAFETY_RETRACTION_TRIGGER` which cancels the current task and moves to a `SAFETY_RETRACT_STATE` where the robot complies gently with the environment and tries to pull up and away from the surface. This transition, and the transition back to the start of the task, have to be included in any state machine you create in order to retain this functionality. See the "SpiralSearch example" code snippets below.
  - Conntact provides `EXIT_STATE` as an easy way to leave the command loop and return control to the user program (that which instantiated the ConnTask). To use this function, always define this state and a transition to it.
  


##### First define the state machine states:
<details><summary>Click to view SpiralSearch example</summary>

```
states = [
START_STATE,
APPROACH_STATE,
FIND_HOLE_STATE, 
INSERTING_PEG_STATE, 
COMPLETION_STATE,
EXIT_STATE,
SAFETY_RETRACT_STATE
]
```



</details>

##### Next define the valid transitions between these states. 

<details><summary>Click to view SpiralSearch example</summary>

```
transitions = [
{'trigger':APPROACH_SURFACE_TRIGGER  , 'source':START_STATE         , 'dest':APPROACH_STATE         },
{'trigger':STEP_COMPLETE_TRIGGER     , 'source':APPROACH_STATE      , 'dest':FIND_HOLE_STATE        },
{'trigger':STEP_COMPLETE_TRIGGER     , 'source':FIND_HOLE_STATE     , 'dest':INSERTING_PEG_STATE    },
{'trigger':STEP_COMPLETE_TRIGGER     , 'source':INSERTING_PEG_STATE , 'dest':COMPLETION_STATE       },
{'trigger':STEP_COMPLETE_TRIGGER     , 'source':COMPLETION_STATE    , 'dest':EXIT_STATE             },
{'trigger':SAFETY_RETRACTION_TRIGGER , 'source':'*'                 , 'dest':SAFETY_RETRACT_STATE,
'unless':'is_already_retracting' },
{'trigger':STEP_COMPLETE_TRIGGER     , 'source':SAFETY_RETRACT_STATE, 'dest':APPROACH_STATE         },
{'trigger':RUN_LOOP_TRIGGER          , 'source':'*'                 , 'dest':None, 'after': 'run_step_actions'}
]
```



</details>

#### 4. Analyze each *step* and determine the *motion profile* and *end conditions.*

For each step above, identify the *force directions* and *free movement directions* required. Define the movement as a [MovePolicy](#MovePolicy) as detailed above.



* To move the peg downward in free space until it hits a hard surface:
  * Apply a small downward force to move the robot downward.
  * Comply in the vertical (z) direction - move as the force assigned above dictates.
  * Do not comply in horizontal directions (x and y); assign the expected hole location to these dimensions.
  * Do not comply in orientation.
  * Exit when a static obstacle stops the robot's motion
  * Save the Z position of the surface so that, later, we can determine if we pass it by falling into the hole.

To realize this behavior in an ConnStep, you can use this MovePolicy in the Step's `__init__` method:

```
self.create_move_policy(move_mode="line",
                        vector=[0,0,1], # Move along Z axis
                        origin=[0,0,0], # Lock X,Y to hole's position
                        force=[0, 0, -7]) # Apply downward force
```

To end the step when collision is detected, simply override the `exit_conditions` method.

```
def exit_conditions(self) -> bool:
    return self.is_static() and self.in_collision()
```

<details><summary>FindSurface as ConnStep</summary>

```
class FindSurface(ConnStep):

    def __init__(self, connTask: ConnTask) -> None:
        ConnStep.__init__(self, connTask)
        # Create a move policy which will move downward along a line
        # at x=0, y=0
        self.create_move_policy(move_mode="line",
                                vector=[0,0,1],
                                origin=[0,0,0],
                                force=[0, 0, -7])
                                
    def exit_conditions(self) -> bool:
        return self.is_static() and self.in_collision()

    def on_exit(self):
        """Executed once, when the change-state trigger is registered.
        """
        # Measure flat surface height and report it to AssemblyBlocks:
        self.assembly.surface_height = self.conntext.current_pose.transform.translation.z
        return super().on_exit()
```

</details>

The spiral search pattern is realized by changing the `MovePolicy`'s origin continuously according to a time-parameterized mathematical formula. We configure the `MovePolicy` with `MoveMode` "Line" so that X and Y will track while we still permit total compliance in `z`, allowing the peg to drop into the hole.

<details><summary>SpiralToFindHole ConnStep</summary>

```
class SpiralToFindHole(ConnStep):
    def __init__(self, connTask: (ConnTask)) -> None:
        ConnStep.__init__(self, connTask)
        self.create_move_policy(move_mode="line",
                                vector=[0,0,1],
                                force=[0, 0, -7])

        self.spiral_params = self.task.connfig['task']['spiral_params']
        self.safe_clearance = self.task.connfig['objects']['dimensions']['safe_clearance']/100 #convert to m
        self.start_time = self.conntext.interface.get_unified_time()
        self.exitPeriod = .05

    @property
    def current_move(self):
        """
        This more complicated motion definition simply updates the origin of the linear move_policy each
        cycle to create a spiralling command position.
        """
        self.move_policy.origin = self.get_spiral_search_pose()
        return self.move_policy.current_move(self.conntext.current_pose.transform.translation)

    def exit_conditions(self) -> bool:
        return self.conntext.current_pose.transform.translation.z <= self.task.surface_height - .0004

    def get_spiral_search_pose(self):
        """Generates position, orientation offset vectors which describe a plane spiral about z;
        Adds this offset to the current approach vector to create a searching pattern. Constants come from Init;
        x,y vector currently comes from x_ and y_pos_offset variables.
        """
        curr_time = self.conntext.interface.get_unified_time() - self.start_time
        curr_time_numpy = np.double(curr_time.to_sec())
        frequency = self.spiral_params['frequency'] #because we refer to it a lot
        #Calculate the amplitude of the spiral:
        curr_amp = self.spiral_params['min_amplitude'] + self.safe_clearance * \
                   np.mod(2.0 * np.pi * frequency * curr_time_numpy, self.spiral_params['max_cycles'])
        x_pos = curr_amp * np.cos(2.0 * np.pi * frequency * curr_time_numpy)
        y_pos = curr_amp * np.sin(2.0 * np.pi * frequency * curr_time_numpy)
        pose_position = [x_pos, y_pos, 0]

        return pose_position
```

</details>

Finally, to follow the peg into the hole, we use a similar setup to the `FindSurface` Step above. This time we allow all axes to comply fully so that the peg experiences minimal friction and aligns more perfectly.

<details><summary>Hole insertion ConnStep</summary>

```
class FindSurfaceFullCompliant(ConnStep):
    def __init__(self, connTask: (ConnTask)) -> None:
        ConnStep.__init__(self, connTask)
        self.create_move_policy(move_mode="free",
                                force=[0, 0, -5])

    def exit_conditions(self) -> bool:
        return self.is_static() and self.in_collision()
```

</details>


That's the majority of the custom programming involved. With this state machine and these Steps, the system can execute the proposed task. You can read through the rest of `SpiralSearch.py` to see how we added plotting capability and used a Connfig to store info about each of the cylindrical pegs in the task board. You can also read through `spiral_search_node.py` to see how we instantiate and run the ConnTask for our ROS+UR10e workcell.

## Running _SpiralSearch_ Example

The repository is currently set up with examples that demonstrate assembly tasks for the [NIST assembly task board](https://www.nist.gov/el/intelligent-systems-division-73500/robotic-grasping-and-manipulation-assembly/assembly) using a Universal Robots UR10e. The system can insert any of the circular pegs into their respective holes. Relative locations of these holes on the board are already recorded in *config/peg_in_hole_params.yaml*.

To run the examples, you must first determine the location and position of the NIST task board (or equivalent). First, by jogging the UR manually, locate the corner of the task board in the robot's base frame. We selected an arbitrary corner to be the board's origin, as shown below.
 
![board_overall](resource/board_overall.jpg)

Align the gripper's axes as marked on the board (align the +X axis with that drawn on the board.)

![board_frame](resource/board_corner.jpg)

Based on the robot's tcp position as displayed on the "Move" tab of the pendant, update the expected position and Z-axis orientation of the board in `peg_in_hole_params.yaml` under `environment state`:
```
environment_state:
    task_frame:
        position: **[-639, 282, -337]**
        orientation: **[0, 0, 0]**
    kitting_frame:
        position: [640, 544, -502]
        orientation: [0, 0, 0]
```
Now the robot can find the rough locations of each of the circular holes. No need to be perfect; the whole system is designed to overcome small inaccuracies! Next, you can select the peg to be inserted by modifying the `task`:
```
task:
    target_peg: "peg_10mm"
    target_hole: "hole_10mm"
    starting_tcp: "tip"
    assumed_starting_height: 0.0
    restart_height: -0.1
```

To run the example, open a terminals sourced to the built project workspace and run:

    roslaunch conntact conntact_demo.launch

## Appendix

###### Interpolation

Our team has implemented a solution to the potential danger of the CCC's setpoint-seeking PD loop. We find that, if the robot is sent to a distant command, the forces and speeds used by the robot to reach that command pose a serious collision danger. It's possible that some limits/parameters can be set in CCC directly to control this, but we didn't find them and needed a quick and certain fix to the rapid unplanned motion problem.

In Conntext's `publish_pose` method, we run the command from the Conntask through an interpolation function `assembly_utils/interp_command_by_magnitude` which limits its *distance in position and orientation* to a user-specified cap.

Results look like this:

![Interpolation](resource/interpolate_big_plot.png)
![Interpolation](resource/interpolate_small_plots.png)

The black arrow indicates the start position (at the base of the arrow) and orientation (the arrow's shaft is X and barb is Z). A trace of intermittent dots link the tip of the arrow to the tip of the target position passed into the interpolation method. The intermediate arrow with the target arrow's color is the resultant interpolated pose. We show the results of limiting the target's offset to within .2m and 20 degrees of the current pose.

The effect of this limiting is to provide a consistent limited speed and force for robot motions. Rotational motions especially are prone to collision, so this was a necessary precaution to permit arbitrary orientation control in ConnStep's MovePolicy.

## Acknowledgements

This project is primarily supported by the National Institute of Standards and Technology through the [Agile Performance of Robotic Systems](https://www.nist.gov/programs-projects/agility-performance-robotic-systems) program under grant award number 70NANB21H018
