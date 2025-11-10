# SLAM Challenge

GMapping implementation.

## How to run
**First** on NoMachine Terminal: 
```
cd ~/mbot_ros_labs/src/mbot_slam/rviz
ros2 run rviz2 rviz2 -d slam.rviz
```
**Second** on VSCode Terminal #1:
```bash
ros2 run mbot_slam slam_node
```

**Last** on VSCode Terminal #2:
```bash
cd ~/mbot_ros_labs/src/mbot_rosbags
ros2 bag play slam_test
```
