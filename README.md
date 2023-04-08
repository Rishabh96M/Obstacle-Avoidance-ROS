# walker
A ROS package for obstacle avoidance of turtlebot3 using the topics /scan and /cmd_vel, Vallidated on Ubuntu 20.04 LTS and ROS Noetic.

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)


## Steps to install and launch the walker
Make a catkin workspace catkin_ws and run the following commands :
  
```
cd <path_to_ws>/catkin_ws/src
git clone https://github.com/Rishabh96M/walker.git
cd ../
catkin_make
```

In a terminal run :
```
roscore
```
This will initialize ROS

In a new terminal run : 
```
source catkin_ws/devel/setup.bash
roslaunch walker walker.launch record:=<bool>
```
This will launch turtlebot3 burger in the gazebo world, start the walker node and record the ros bag file as per command

Note:- Replace <bool> by either 'true' or 'false' depending on if you want to record the session for 15s
Example:
```
roslaunch walker walker.launch record:=true
```

Note: The recorded file will be saved in **results** folder with the name "walker_results.bag"

## Checking the information of bag file
To check the information of the bag file, run the following command in the location where bag file is saved:
```
rosbag info walker_results.bag
```

## Playing from the bag file
With roscore and the listener node running, in a new terminal run:
```
source catkin_ws/devel/setup.bash
rosbag play rosbag_chatter_results.bag
```

## Running cpplint & cppcheck tests
Run the following command in the src directory of the project to generate cpplint results, the ouput is stored in the **results** folder
```
cpplint $( find . -name \*.h -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/")
```
Run the following command in the src directory of the project to generate cppcheck results, the ouput is stored in the **results** folder
```
cppcheck --language=c++ --std=c++11 -I include/ --suppress=missingIncludeSystem  $( find . -name \*.h -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/")
```

The **results** folder contains the results of cpplint and cppcheck.
