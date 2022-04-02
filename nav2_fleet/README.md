# Nav2 Fleet (Python3)

## Overview

This package provides a simulation of a fleet of robots in a a warehouse environment (AWS RoboMaker Small Warehouse World). The simulation can include up to 4 robots. Tasks are regularly sent to the robots : go to a shelf to pick an item and ship it at a specified location. This project is based on ROS2 and uses Nav2 framework for robots navigation. This was built by [Pierre Herent](https://www.linkedin.com/in/pierre-hv/).

![](media/readme.gif)

## Requirements

To build this package you will need [ROS2](https://docs.ros.org/en/galactic/Installation.html), [Nav2](https://navigation.ros.org/build_instructions/) with [AWS RoboMaker Small Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/tree/ros2) built in a local workspace (this workspace must contain `navigation2` and `aws-robomaker-small-warehouse-world` alongside). The simulation uses Gazebo simulator and [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) models need to be installed.

## Installation 

To install this package, just copy the `nav2_fleet` directory in `nav2_ws/src/navigation2` where `nav2_ws` is the name of your Nav2 workspace. Then build it with :

``` bash
# Build the Nav2 packages
colcon build --symlink-install
```

You are now ready to use the package. 

## Usage of the simulation

Open a new terminal (separate from the one used to build) and source the local workspace. You also need to export Turtlebot3 model : 
``` bash
cd nav2_ws
. install/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<distro>/share/turtlebot3_gazebo/models

# Run the simulation environment :
ros2 launch nav2_fleet wh_multi_tb3_simulation_launch.py world:=/your_path/nav2_ws/install/nav2_fleet/share/nav2_fleet/warehouse.world map:=/your_path/nav2_ws/install/aws_robomaker_small_warehouse_world/share/aws_robomaker_small_warehouse_world/maps/005/map.yaml n_robots:=3 
```
`n_robots` is an optional parameter to set the number of robots. Maximum is 4, default is 2.

Once Gazebo has launched and the robots have spawned, open a new terminal and run :

``` bash
# n should be the same as n_robots
ros2 run nav2_fleet multi_picking -n 3
```

After a few seconds, the robots should move !

## Notes 

The Python program that assigns tasks and manage the robots is `nav2_fleet/nav2_fleet/multi_picking.py`. 
The program doesn't handle robots collisions yet : they are likely to bump each other. This problem will be solved in a future update.
The program 
