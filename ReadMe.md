# ConnTact

Software framework to enable agile robotic assembly applications.

(Connect + Tactile)

## Overview

![Framework diagram](resource/framework.png)

The ConnTact package provide a efficient framework for assembly algorithm development for compliant robots. It allows the user to define fully tactile methods for making tight-tolerance connections in a human-like way.

ConnTact includes an implementation of the [transitions](https://github.com/pytransitions/transitions) state machine package to define the steps and decisions critical to your algorithm. It also provides a suite of tools and examples to sense the environment based on force feedback - detecting collision, hard surfaces, and position changes. Finally we define some basic motion profiles which can be used to probe the environment and align the tool to the workpiece.

## Installation

Development of framework was done under Ubuntu Focal (20.04) using ROS Noetic.

  - Clone the repository into the `src/` directory of a ROS workspace (e.g., `~/ros_ws/src`)
  - Install Python dependencies `pip install transitions modern_robotics`
  - Install ROS source dependencies:
    - `cd ~/ros_ws/src`
    - `vcs import < conntact/dependencies.rosinstall`
  - Install ROS package dependencies: `rosdep install --rosdistro noetic --ignore-src --from-paths .`
  - Source ROS and build the workspace: `. /opt/ros/noetic/setup.bash`, `catkin build`


## Usage

### Structure

Conntact is structured with 3 levels of functionality:

#### AssemblyTools

> *Do the following job* ***here*** *and at* ***this*** *angle*

  The AssemblyTools package provides a bunch of utility functions. These translate data from hardware into **task space**: the frame-of-reference of the task to be accomplished. This way, an algorithm developed at a given position and orientation is automatically reoriented and repositioned when a new task position is fed into the system. AssemblyTools also provides useful data, such as speed estimation and sensor data filtering.

#### AlgorithmBlocks

> *First do* ***this*** *and then do* ***this*** *and then* ***this***

  The AlgorithmBlocks program is an implimentation of the pytransitions Machine package. It comprises example functions which run according to an easily-reconfigurable state machine. By breaking a task down into sequential tasks, you can use AlgorithmBlocks to sequence these tasks and add decision-making/redundancy/fallback functionality.

#### AssemblyStep

> *Do* ***this*** *until* ***this*** *happens*

  AlgorithmBlocks can run *functions* for simple tasks, but for tactile sensing you often need a lot of specific variable and flags to track conditions. These tasks are streamlined with the AssemblyStep class. Each Step object provides all the basic functions for a sensing task - setup, loop behavior, completion checking - and can be easily expanded or overriden to accomplish a wide variety of tasks.

### Development

We suggest the following workflow to take a task from a human and give it to a robot.

### Setting up a workcell

### Configuring a new application


## Running Examples

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
Currently two algorithms are implemented that perform peg insertion tasks: a vertical searching method and a corner-contact searching method.

To run these examples, open a terminals sourced to the built project workspace and run:

    roslaunch conntact ur10e_upload_compliance.launch algorithm_selected:=<algorithm>

Where `<algorithm>` is either `spiral_search_node` or `corner_search_node`*.

> ⚠ **Note:** Corner Search mode is WIP. This program, or any other which changes the robot TCP orientation, currently causes a rapid motion to assume the specified orientation which can cause collision or damage or just scare you out of your pants. We recommend leaving the orientation vertical until this problem is solved.  

## Acknowledgements

This project is primarily supported by the National Institute of Standards and Technology through the [Agile Performance of Robotic Systems](https://www.nist.gov/programs-projects/agility-performance-robotic-systems) program under grant award number 70NANB21H018
