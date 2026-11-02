# AprilTag search, tracking and hover with ROS 2

A ROS 2/MAVROS mission project for an X500-class UAV. It connects a search path, AprilTag detections, proportional XY tracking and mission-state transitions:

```text
TAKEOFF → SEARCH → TRACK → HOVER
             ↑       │
         lawnmower   tag-pose updates
```

The repository contains the mission implementation, three control-utility tests and a deterministic offline tracking exercise. End-to-end PX4 SITL/HITL mission execution has not been verified.

## Components

- **Search planner:** generates a lawnmower sweep over configured bounds.
- **AprilTag bridge:** converts detector output into mission target-pose/state topics.
- **Tracking controller:** generates proportional XY velocity commands with speed saturation.
- **Mission manager:** handles takeoff, search, tracking and final hover.
- **MAVROS interface:** connects the mission to flight-controller pose/setpoint interfaces.

GPS-denied operation expects an external localization source on `/vio/pose`. This project does not implement VIO, SLAM, obstacle avoidance or multi-tag mapping.

## Offline checks

Run from the repository root; these checks do not require a ROS runtime:

```bash
python3 -m pytest -v ros_uav_apriltag/tests/test_control_utils.py
PYTHONPATH=ros_uav_apriltag python3 ros_uav_apriltag/tests/run_offline_validation.py
```

The three collected tests cover lawnmower waypoint generation, velocity saturation and proportional velocity commands. They can also be run as a script:

```bash
PYTHONPATH=ros_uav_apriltag python3 ros_uav_apriltag/tests/test_control_utils.py
```

The offline mission injects a synthetic detection, transitions from search into tracking, and reaches hover with a final XY error of approximately **0.184 m** against a **0.2 m** tolerance. Assertions check that hover is reached and that the final error is inside tolerance before the runner reports success.

This is a numerical control exercise. It does not test camera detection, external VIO, MAVROS communication or multirotor flight dynamics. See [verification details](ros_uav_apriltag/TEST_RESULTS.md).

## Build in a ROS 2 workspace

With ROS 2 and the required MAVROS/AprilTag dependencies installed:

```bash
mkdir -p ~/apriltag_ws/src
cd ~/apriltag_ws/src
git clone https://github.com/sumangowdavr/UAV-Tracking-using-IBVS-and-AprilTag.github.io.git
cd ..
colcon build --packages-select ros_uav_apriltag
source install/setup.bash
```

## Run

```bash
ros2 launch ros_uav_apriltag x500v2_gps_denied_apriltag.launch.py \
  fcu_url:=udp://:14540@127.0.0.1:14557
```

Required inputs:

- `/vio/pose`
- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `/tag_detections`

## Configuration

In `ros_uav_apriltag/config/mission.yaml`, set the search bounds and lane spacing, takeoff/hover altitudes, tracking gain, XY speed limit and centering tolerance. The launch file connects the external VIO pose with the MAVROS vision-pose input.

The next integration milestone is a repeatable PX4/MAVROS simulation with camera detections, localization and a recorded mission trace.
