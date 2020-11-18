
**NOTE:** The code in this repository is being updated/uploaded and only initial versions of some of the ROS nodes are available at the moment. For details about the full implementation (will be available soon), please visit:

[https://tiers.utu.fi/paper/queralta2020viouwb](https://tiers.utu.fi/paper/queralta2020viouwb)

or arXiv

[https://arxiv.org/abs/2011.00830](https://arxiv.org/abs/2011.00830)

# Graph Rigidity with UWB ranging

ROS Nodes for collaborative localization and graph rigidity based on UWB ranging.

This code has been used for collaborative localization between UGVs and UAVs under the following conditions
- UWB ranging is possible between any pair of robots (default topics are `/uwb/ranging/{{ robot_one_hostname }}/{{ robot_two_hostname }}`).
- If not all UWB transceivers are on the same horizontal plane, then the relative altitude is also available (by default measured with TFmini lidar and available on topic `/{{ robot_hostname }}/tfmini_ros_node/range`).
- Ground robots are equipped with 3D lidars and able to rotate and detect the UAVs (by default lidar data is published to topic `\{{ robot_hostname }}/lslidar/scan`)
- Ground robots with limited-FoV Livox lidars publish `\{{ robot_hostname }}/livox/lidar)
- Robots share a common global orientation frame (e.g., from a magnetometer). In practice this can be achieved by initializing all robots with the VIO camera looking in the same direction.

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

For any questions, write to `jopequ@utu.fi`.

Visit us at [https://tiers.utu.fi](https://tiers.utu.fi)
