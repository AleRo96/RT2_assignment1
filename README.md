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

First of all, make sure to source the workspace with the right ROS distro.
To launch the simulation with Gazebo, please run:
```
roslaunch rt2_assignment1 sim.launch
```
In the lauch folder, there is anothere file (Coppelia.launch) that won't open the Gazebo environment.
If you desire to open this lighter simulation, just run:

```
roslaunch rt2_assignment1 coppelia.launch
```
In both cases, using either of the lauch files, the regular one or the slimmer one, you have to open the Coppelia.sh in another shell e loading
the "rt2_scene.ttt" file present in this brach.
