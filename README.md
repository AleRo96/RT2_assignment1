# First And Second Assignment of the Research Track 2 course (Robotics Engineering / JEMARO, Unige)
The package contains the nodes and the simulation environment for controlling a mobile robot in the Gazebo and Coppelia simulation environment.

- docs: Folder which contains the documentation generated using doxygen.
- launch: Folder which contains "launch.py" file. 
- src: Folder which contains the cpp files "randomserver.cpp" and "computer.cpp".
- srv: Folder which contains the messages used by the servers. 
- CMakelists.txt: File to build the workspace
- package.xml
- mapping_rule.yaml: File which contains the mapping rule for the bridge between ros1 and ros2
- README.md: This file containing information about the package

# HOW TO RUN:

First of all, you need to build both the ros1 workspace and ros2 workspace.\\
Once you have done that, you can source them using the right ROS distros.
It's mandatory to have installed the ros1_bridge package in your ROS 2 workspace.

In a shell, source the ROS 1 distro and launch the simulation with Gazebo:
```
roslaunch rt2_assignment1 sim.launch
```
In a second shell, source both the distros (ROS12.sh) by using the bridge. Then run:
```
ros2 run ros1_bridge dynamic_brdge
```
In a third shell, source the ROS 2 distro and run:
```
ros2 launch rt2_assignment1_ros2 launch.py
```
