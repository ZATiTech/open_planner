# open_planner
## Carla Simulator - Autoware Universe Bridge nodes

## Consists of: 
- carla_sensor_kit
- carla_pointcloud
- op_fix2pose

## Support: 
- CARLA Simulator 0.9.15 release 
- Autoware Universe (master) December 2023
- Python 3.10 
- ROS2 Humble
- OpenPlanner.Universe
- Linux 22.04

## Useful Links: 
- Announcement on Autoware discussions: 
https://github.com/orgs/autowarefoundation/discussions/2828

- op_agent (ros2-humble branch)
https://github.com/hatem-darweesh/op_agent/tree/ros2-humble

- op_bridge (ros2-humble branch)
https://github.com/hatem-darweesh/op_bridge/tree/ros2-humble

## System Installation
1- Ubuntu 22.04 
2- Install Autoware.universe with all its requirements. Use "./setup-dev-env.sh" [from](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/) 
3- Make sure these tutorials are working after autoware build
4- Clone OpenPlanner in the autoware.universe folder (/src/universe/external) 
5- Clone additional [LIDAR driver](https://github.com/autowarefoundation/awf_velodyne) in the folder (/src/universe/external)
6- Copy the files (sensor_kit_calibration.yaml, sensors.calibration.yaml) from folder "open_planner/carla_sensor_kit_launch/carla_sensor_kit_description/config" to "src/param/autoware_indicidual_params/carla_sensor_kit"
6- Rebuild autoware.universe

