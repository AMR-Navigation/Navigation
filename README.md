# Navigation Project

## Preparation
In order to prepare your environment to run this code you will need to install four things: (1) ROS 1 Melodic, (2) Gazebo 9, (3) the Turtlebot3 simulation files, and (4) Python 3 in parallel with Python 2. You will also need an appropiate version of Ubuntu that is compatible with those three installations. Once you have the necessary files and libraries installed and you have verified that they work together, you can copy all the files in the src directory here into your own src directory. Finally, run `catkin_make` to build everything.

## Overview
The navigation stack that we have built includes many parts, so there are several steps to starting it up. Aside from the typical parts of a ROS launch with Gazebo, the stack includes
1. The controller package, which handles laser processing
2. A Python server that allows the Python 2 ROS software to communicate with the Python 3 YOLOv8 model
3. The yolo package, which handles camera processing
4. The fusion package, which fuses the processed data from the controller and yolo packages
5. The pathing plugin, which specifies a new layer for the costmap_2d package in the default turtlebot3 navigation stack

## Starting the navigation stack
**First**, launch a ROS file that will start Gazebo and spawn the turtlebot3 waffle_pi robot. For example, try running

`export TURTLEBOT3_MODEL=burger`

and then

`roslaunch turtlebot3_gazebo navigation.launch`
.

**Second**, open a new terminal, navigate into /src/yolo/src, and run `server.py` *with Python 3*. For example,

`cd src/yolo/src`
`python3.7 server.py`

This will start the server that allows the ROS Python 2 code to communicate with the YOLOv8 model in Python 3.


**Third**, open a new terminal and launch the controller, yolo, and fusion nodes either by running

`rosrun controller src/controller.py`
`rosrun yolo src/yolo_node.py`
`rosrun fusion src/fusion.py`

in separate terminals or creating a launch file for all three of them like 

```
<launch>
	<node name="controller" pkg="controller" type="controller.py" output="screen"></node>
	<node name="yolo" pkg="yolo" type="yolo_node.py" output="screen"></node>
	<node name="fusion" pkg="fusion" type="fusion.py" output="screen"></node>
</launch>
```

and running it. At this point you should have gazebo open, as well as the laser gui from the controller node and possibly a gui with camera output from yolo. Additionally, check the server output and make sure it is receiving image queries. 

**Fourth**, start the default turtlebot3 navigation stack in a new terminal with 

`roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml`

and then start the plugin with 

`roslaunch pathing path.launch`


## Diagnosing problems
If the navigation stack does not seem to be working but there are no obvious errors, then one of the components is probably not functioning correctly. In order to identify what might be going wrong, ensure the following:

1. The gazebo environment looks correct.
2. The gui for `controller` is up and the node is publishing on the `LidarList` topic.
3. The yolo server is running and receiving input (`Received  x  bytes`).
4. The yolo node is running and publishing detections to the `object_detection_results` topic when there is something to detect.
5. The fusion node is running and is publishing to `coordinates_topic` when there are detections.
6. The pathing plugin is generating output when data is published to `coordinates_topic`.

## The static environment
The static environment we used for experiments can be found in the `worlds` directory. To use it, copy it into the worlds directory of the `turtlebot3_gazebo` package and reference it in launch files with `$(find turtlebot3_gazebo)/worlds/static_navigation.world`. 

## Launching experiments
Most of the experiment launch files can be found in the my_gazebo_sim package. For a given experiment, launch the environment file (e.g. `roslaunch my_gazebo_sim 1_person/1_person.launch`), prepare whatever navigation stack you are testing, and then launch the experiment file (e.g. `roslaunch my_gazebo_sim 1_person/1_person.launch`). You can also create your own experiments in a similar fashion. 
