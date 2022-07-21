# 3D Mapping For Autonomous Robots
This repo contains 3D [octomap](https://octomap.github.io/), [occupancy map](https://en.wikipedia.org/wiki/Occupancy_grid_mapping) and ESDF map for autonomous navigation.

### Install
```
sudo apt-get install ros-noetic-octomap*
git clone https://github.com/Zhefan-Xu/map_manager.git
cd ~/catkin_ws
catkin_make
```
### Run demo (simulatation)
Please intall package [uav_simulator](https://github.com/Zhefan-Xu/uav_simulator.git), then run the occupancy map launch: 
```
roslaunch uav_simulator start.launch
roslaunch map_manager occupancy_map.launch
roslaunch map_manager rviz.launch
```

or run ESDF map:
```
roslaunch uav_simulator start.launch
roslaunch map_manager esdf_map.launch
roslaunch map_manager rviz.launch
```
