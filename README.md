# Clean_Robot

## Introduction

The codes base on ROS Ubuntu 18.04 or Ubuntu 20.04. You can download and copy it to your ROS workspace, and run:

> catkin_make

The repository provides some algorithms for indoor sweeping robots, which contain four functional packages:
- full_coverage: Full coverage path planning, your robot will plan the path independently and clean the whole map.
- multi_goals: With this package, you can interactively select target points on the map by Rviz, and then your robot will move to the points.
- mw-exploration: Multi-weight exploration algorithms, based-frontier exploration.
- show_trajectory: Show robot's trajectory on the map.

## Usage

### 1.mw-exploration

Run the following code to load the robot model and environment information:

> roslaunch mw-exploration indoorshed.launch

Run the following code to start gmapping(SLAM), move-base(path planning) and Rviz(visualization):

> roslaunch mw-exploration slam_view.launch

Use the following code to run based-frontier exploration to build the whole map:

> roslaunch mw-exploration explore.launch

and then your can use the Rviz tool--'Point' to click some points to build a closed polygon region, your robot will only move in this area. 

The score function is as follows:

$$C(x) = w_d \frac{1}{D(x_r,x)}+w_o \frac{1}{O(x_r,x)}+w_s S(x)+w_i I(x)$$

where, x denotes the candidate frontier point, $x_r$ denotes the current robot's position, C(X) denotes the score, $D(x_r,x)$ denotes the distance between the candidate frontier point and robot, $O(x_r,x)$ denotes the orientation between the candidate frontier point and robot, S(x) denotes the size of the candidate frontier point, I(x) denotes the anticipatory information gain, and $w_d,w_o,w_s,w_i$ denote the corresponding weight.

Finally, your can run map_server in "map" catalogue to save the result:

> rosrun map_server map_saver -f result

An example of a created map is as follows:

<div align=center>
<img src="https://github.com/Yaepiii/Clean_Robot/assets/75295024/0b68cd66-7c0b-4cb2-b8fe-0ba519a71868" width="300" height="300"/>
</div>

### 2.full_coverage

Before running this feature pack, make sure you have put your environment map in the "map" folder of this feature pack!

Run the following code to load the robot model and environment information:

> roslaunch full_coverage indoorshed.launch

Run the following code to start the full coverage path planning, which has included AMCL(location), move-base(path planning) and Rviz(visualization):

> roslaunch full_coverage clean_work.launch

With this algorithm, you may see the following result:

<div align=center>
<img src="https://github.com/Yaepiii/Clean_Robot/assets/75295024/ba0ca50b-4192-4a44-82de-ab35873accee" width="300" height="300"/>
</div>

### 3.multi_goals

Before running this feature pack, make sure you have put your environment map in the "map" folder of this feature pack!

Run the following code to load the robot model and environment information:

> roslaunch full_coverage indoorshed.launch

Run the following code to start AMCL(location), move-base(path planning) and Rviz(visualization):

> roslaunch multi_goals slam_view_amcl.launch

Run the following code to start multi-target points path planning, and you can use the Rviz's tool 'Point' select some points that you expect your robot move to:

> roslaunch multi_goals multi_goals.launch

With this algorithm, you may see the following result:

<div align=center>
<img src="https://github.com/Yaepiii/Clean_Robot/assets/75295024/63754c0c-1bd2-45da-84d3-e167bc9b987c" width="300" height="300"/>
</div>

