# Graph Rigidity with UWB ranging

ROS Nodes for collaborative localization and graph rigidity based on UWB ranging.

This code has been used for collaborative localization between UGVs and UAVs under the following conditions
- UWB ranging is possible between any pair of robots (default topics are `/uwb/ranging/{{ robot_one_hostname }}/{{ robot_two_hostname }}`).
- If not all UWB transceivers are on the same horizontal plane, then the relative altitude is also available (by default measured with TFmini lidar and available on topic `/{{ robot_hostname }}/tfmini_ros_node/range`).
- Ground robots are equipped with 3D lidars and able to rotate and detect the UAVs (by default lidar data is published to topic `\{{ robot_hostname }}/lslidar/scan`)
- Robots share a common global orientation frame (e.g., from a magnetometer)

## Installation

This instructions are for Ubuntu 18.04 with ROS Melodic already installed.

Clone this repo into your catkin_ws (the code below creates a new catkin workspace named `local_ws` in your home folder):
```
mkdir -p  ~/local_ws/src && cd ~/local_ws/src
git clone https://github.com/TIERS/uwb-graph-rigidity-ros.git
```

Then build the workspace. We recommend using `catkin build`. Install it if needed with
```
sudo apt install python-catkin-tools
```

then run
```
cd ~/local_ws
catkin init
catkin build
```

## Dependencies

We use PCL to extract the UAV positions from ground robots. Install
```
sudo apt install ros-melodic-pcl-ros ros-melodic-pcl-msgs ros-melodic-pcl-conversions libpcl-dev

```
## Usage

TODO

## Contact

TODO
