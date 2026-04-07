# ROS2 UAV AprilTag Mission (GPS-denied)

This project is a ROS2 mission stack for X500V2 + MAVROS that performs the full flow:

**search → detect → track → hover**

## Mission behavior

1. **Take off** in local frame using OFFBOARD position setpoints.
2. **Search planner** generates a lawnmower sweep across a defined area.
3. **AprilTag detector** (`apriltag_ros`) watches camera feed and estimates tag pose.
4. **AprilTag bridge** converts detections into mission target pose/state topics.
5. **Tracking controller** commands XY velocity to center over the detected tag.
6. **Mission controller** switches from SEARCH to TRACK to final HOVER.

## Components

- **MAVROS interface**: bridges ROS2 commands to flight controller.
- **Mission controller**: phase logic (takeoff/search/track/hover).
- **AprilTag detector**: camera-based tag detection (`apriltag_ros`).
- **Search planner**: area sweep waypoints (lawnmower).
- **Tracking controller**: proportional velocity alignment to tag.

## Not included (by design)

- obstacle avoidance
- multi-tag mapping
- full SLAM/VIO implementation (expects external `/vio/pose`)
- real-hardware PID tuning for your exact vehicle

## Build

```bash
cd <ros2_ws>/src
ln -s /workspace/welcome.github.io/ros_uav_apriltag .
cd ..
colcon build --packages-select ros_uav_apriltag
source install/setup.bash
```

## Run

```bash
ros2 launch ros_uav_apriltag x500v2_gps_denied_apriltag.launch.py \
  fcu_url:=udp://:14540@127.0.0.1:14557
```

## Required inputs

- `/vio/pose` (relayed to `/mavros/vision_pose/pose` for GPS-denied localization)
- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `/tag_detections`

## Config

Tune mission in `ros_uav_apriltag/config/mission.yaml`:

- `search_planner.bounds`, `lane_spacing_m`
- `tracking_controller.kp_xy`, `max_xy_vel_mps`, `center_tolerance_m`
- `mission_manager.takeoff_altitude_m`, `hover_altitude_m`, `center_hold_time_s`

## Test results in this container

What I tested here:

- Python compile checks for all nodes and launch/setup files.
- Deterministic control utility checks (lawnmower path + velocity saturation).
- XML parse check for `package.xml`.

What I could not run here:

- End-to-end PX4 SITL/HITL flight simulation (runtime stack unavailable in this container).

So this commit provides the full ROS2 project implementation and static/unit validation, but you must run SITL/HITL mission execution in your PX4+ROS2 environment for true flight results.

Detailed run artifacts are recorded in `ros_uav_apriltag/TEST_RESULTS.md`.
