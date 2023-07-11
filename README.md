# First And Second Assignment of the Research Track 2 course (Robotics Engineering / JEMARO, Unige)
The package contains the nodes and the simulation environment for controlling a mobile robot in the Gazebo and Coppelia simulation environment.

- action: Folder which contains the action's structure.
- docs: Folder which contains the documentation generated using doxygen.
- launch: Folder which contains "sim.launch" and "sim_coppelia.launch" file. 
- notebooks: Folder which contains the file "WidgetRobotControl.ipynb".
- scripts: Folder which contains the python files "go_to_point.py" and "user_interface.py".
- src: Folder which contains the cpp files "position_service.cpp" and "state_machine.cpp".
- srv: Folder which contains the messages used by the servers. 
- urdf: folder which contain a file about the robot information.
- CMkelists.txt: File to build the workspace
- index.html
- package.xml
- README.md: This file containing information about the package
- rt2_scene.ttt: Coppelia sim file with the scene to simulate

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
In both cases, using either of the lauch files, the regular one or the slimmer one, you have to open the Coppelia.sh in another folder e loading
the "rt2_scene.ttt" file present in this brach.
