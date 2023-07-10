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

# HOT TO RUN:

First of all, make sure to source your workspace.
To launch the simulation with Gazebo, please run:
```
roslaunch rt2_assignment1 sim.launch
```

To launch the simulation with Coppelia, please run:
```
roslaunch rt2_assignment1 coppelia.launch
```
This version won't open Gazebo simulator.
Thus, open Coppeliasim.sh in another shell and load the rt2_scene.ttt present in the main brach.
