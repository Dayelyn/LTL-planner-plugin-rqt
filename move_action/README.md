# ros_move_base_action

A ros package that can navigate robots without help of RViz

## Description

The project is part of another project <repository>, aiming at move and navigate a robot(turtlebot in gazebo under this case) without the help of RViz. The navigation goal sequence is calculation with a LTL(Linear Temporal Logic) planner from [P_MAG_TS](https://github.com/MengGuo/P_MAS_TG). When a goal sequence has been planned, it will be sent to move_base action server built in ROS and execute goals one by one. 

## Getting Started

The following step is an instruction of how to deploy the package on your computer.

### Prerequisites

* Ubuntu 14.04 with ROS indigo installed
* The package is based on turtlebot, full turtlebot package should be installed (including simulation environment). Please follow [Turtlebot Instruction](http://wiki.ros.org/Robots/TurtleBot) 
* [P_MAG_TS](https://github.com/MengGuo/P_MAS_TG) is used. Please follow the [instruction](https://github.com/MengGuo/P_MAS_TG/blob/master/README.md).
* If you have failed in running test code in P_MAG_TS, please use 1.x version of networkx.

### Installing

Download or pull the code into your ROS workspace

```
cd ~/your_ROS_workspace
catkin_make
```

## Running the tests

After ```catkin_make``` you could run

```
roslaunch move_action move_navi.launch
```

Gazebo is running and you should see a turtlebot is spawn in gazebo. Then open a new terminal and run

```
rosrun move_action state_publisher.py
```

Normally you will see the robot rotate for a while and then move to a point and go back. 
* The map properties is predefined in plan_service.py
* The task is predefined in task_publisher.py. You can modify by yourself.

### Mechanism Explaination

* task_publisher.py publishes a predefined LTL task. For more information about LTL task, please follow [instruction](https://github.com/MengGuo/P_MAS_TG/blob/master/README.md).
* plan_service.py is a ROS service return a pose sequence based on LTL task. It is calculated by [P_MAG_TS](https://github.com/MengGuo/P_MAS_TG).
* move_acton_service.py is a ROS service synthesizing the pose sequence and linking the sequence with move_base action server. The node is simply based on actionlib of ROS, you can get further infomation at [ROS Wiki](http://wiki.ros.org/actionlib).
* state_publisher.py is simply designed for testing service call.
* You can modify plan_service.py as your own planner.

## Acknowledgments

* The package [P_MAG_TS](https://github.com/MengGuo/P_MAS_TG) from MengGuo is really appreciated. It is the core work for this project.
* The idea is inspired from [FiorellaSibona](http://www.hotblackrobotics.com/en/blog/2018/01/29/action-client-py/), thanks for her explicit tutorial.
* [PurpleBooth](https://gist.github.com/PurpleBooth/109311bb0361f32d87a2#file-readme-template-md) provides brilliant templates of README.md
 

