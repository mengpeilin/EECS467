# Setpoint challenge

ROS 2 package for motion control, trajectory following, and debugging.

## Package Contents

### Executables
- `square_publisher.cpp`: Publishes goal waypoints to `/goal_pose_array` topic. Configured to publish a square trajectory by default; can be modified to publish any custom path.
- `odom.cpp`: Odometry node that publishes odometry estimates to `/odom` topic and TF tree.
- `motion_controller_diff.cpp`: Differential motion controller that follows waypoint goals. Logs debug data to CSV for analysis.

### Tools
- `motion_controller_plot.py`: Post-run analysis tool that plots PID errors and velocity commands from the CSV log generated during motion control.
    ```bash
    python3 motion_controller_plot.py
    ```

## Build

```bash
colcon build
source install/setup.bash
```

## Run
```bash
ros2 run mbot_setpoint square_publisher
```
- This will publish waypoints

```bash
ros2 run mbot_setpoint odom
```
- This will publish odom, and odom -> base_footprint

```bash
ros2 run mbot_setpoint motion_controller_diff
```
- This will start motion_controller