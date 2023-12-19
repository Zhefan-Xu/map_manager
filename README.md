# 3D Mapping Library For Autonomous Robots
This package is a library implementing various 3D mapping algorithms, such as occupancy voxel map, ESDF Map, [dynamic map](https://ieeexplore.ieee.org/abstract/document/10161194) (our mapping for dynamic environments), for autonomous mobile robots. 

```
sudo ln -s /usr/include/eigen3/Eigen/ /usr/include/Eigen
```

## I. Installation Guide
This repo has been tested on ROS Melodic with Ubuntu 18.04 and ROS Noetic with Ubuntu 20.04 and it depends on [onboard_detector](https://github.com/Zhefan-Xu/onboard_detector) which provides the dynamic obstacle detection and tracking for our [dynamic map](https://ieeexplore.ieee.org/abstract/document/10161194). 

```
git clone https://github.com/Zhefan-Xu/onboard_detector.git
git clone https://github.com/Zhefan-Xu/map_manager.git

cd ~/catkin_ws
catkin_make
```
### II. Run DEMO 
a. **Occupancy Map:** In case you do not have/want a hardware platform to play with this repo, we have provided a lightweight [simulator](https://github.com/Zhefan-Xu/uav_simulator.git) for testing. Run the following command to launch the occupancy voxel map:

```
roslaunch uav_simulator start.launch
roslaunch map_manager occupancy_map.launch # check the launch file for ESDF map and dynamic map
roslaunch map_manager rviz.launch
```

The example of running occupancy map is shown as below (note that the robot is controlled by keyboard):

https://github.com/Zhefan-Xu/map_manager/assets/55560905/499a84ad-105e-4907-b7d5-2d4fe59a0894

b. **ESDF Map:** The ESDF map is built upon on the occupancy map. The following command will launch the ESDF map:

```
roslaunch map_manager ESDF_map.launch
```
The example screenshot of ESDF map is shown below which visualizes the distance field to obstacles:

![ESDF](https://github.com/Zhefan-Xu/map_manager/assets/55560905/e37243c6-eefe-4824-800d-1d8b35aaa74b)

c. **Dynamic Map:** The dynamic map integrates a dynamic object detector to represent both static and moving obstacles in dynamic environments. The following command will launch the dynamic map:


```
roslaunch map_manager dynamic_map.launch
```

The example of the dynamic map is shown below: 

![Screenshot from 2023-12-19 01-07-14](https://github.com/Zhefan-Xu/map_manager/assets/55560905/e9575308-c18f-49b0-9ed3-f5946478c8f5)

The related paper can be found on:

**Zhefan Xu\*, Xiaoyang Zhan\*, Baihan Chen, Yumeng Xiu, Chenhao Yang, and Kenji Shimada, "A real-time dynamic obstacle tracking and mapping system for UAV navigation and collision avoidance with an RGB-D camera”, IEEE International Conference on Robotics and Automation (ICRA), 2023.** [\[paper\]](https://ieeexplore.ieee.org/abstract/document/10161194) [\[video\]](https://youtu.be/u5zblVx8KRc?si=3c2AC9mc6pZBUypd).


### III. Parameters
Please find parameters in ```map_manager/cfg/***.yaml``` files.

### IV. ROS Topics
Subsribe the following topics for occupancy and ESDF map:
  - Localization topic: ```robot/odometry``` or ```robot/pose``` (please enter the name of your topic in the parameter files)
  - Depth camera topic: ```camera/depth``` (defined in the config file)
  
Publish the following topics:
  - occupancy map visualization: ```occupancy_map/inflated_voxel_map```
  - esdf map visualization: ```esdf_map/inflated_voxel_map``` and ```esdf_map/esdf```

### V. C++ Code API
Example code:
```
#include <map_manager/ESDFMap.h>

int main(){
  ...
  map_manager::ESDFMap m;
  m.initMap(nh);
  
  // collision checking given a point
  Eigen::Vector3d pos (1.0, 1.0, 1.0)
  bool hasCollision = m.isOccupied(pos);
  
  // get distance with gradient
  Eigen::Vector3d grad;
  double dist = m.getDistanceWithGradTrilinear(pos, grad);
  ...
}
```
### VI. Example
ESDF Map: ![Screenshot from 2022-07-21 15-48-26](https://user-images.githubusercontent.com/55560905/180302896-ee4a9a80-4aac-4cff-8425-7fe42a45b827.png)
Occupancy Map Video:


https://user-images.githubusercontent.com/55560905/180305608-402a8e33-f2d6-40fd-9c66-18e610a120ac.mp4



