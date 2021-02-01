# pendulum
Prototype Flywheel Pendulum

## Setting up project
1. create directory catkin_ws/src/
2. clone repo into it

## Running Simulation
1. build (need to do it first time after clone, and any time you modify c++ files)
```
cd ~/catkin_ws
catkin build
```
2. source (do it for each new terminal)
```
source build/setup.bash
```
3. launch simulation
```
roslaunch pendulum gazebo.launch
```
