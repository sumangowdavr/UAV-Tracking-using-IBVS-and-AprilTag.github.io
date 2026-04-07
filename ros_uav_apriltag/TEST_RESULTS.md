# Test Results

Date: 2026-04-07 (UTC)

## Offline validation run

Command:

```bash
PYTHONPATH=ros_uav_apriltag python3 ros_uav_apriltag/tests/run_offline_validation.py
```

Output:

```text
offline_validation: PASS
waypoint_count=8
detect_step=40
centered_step=93
final_xy_error_m=0.184
```

Interpretation:
- Synthetic mission simulation transitions SEARCH -> TRACK -> HOVER.
- Tracking controller converged to within 0.2 m tolerance in the model.

## Static checks

```bash
python3 -m py_compile ros_uav_apriltag/ros_uav_apriltag/*.py ros_uav_apriltag/launch/x500v2_gps_denied_apriltag.launch.py ros_uav_apriltag/setup.py
PYTHONPATH=ros_uav_apriltag python3 ros_uav_apriltag/tests/test_control_utils.py
python3 - <<'PY'
import xml.etree.ElementTree as ET
ET.parse('ros_uav_apriltag/package.xml')
print('xml_ok')
PY
```

All passed in this container.

## Runtime limitation

`ros2` CLI/runtime is not installed in this container, so full ROS2+PX4 SITL launch and flight trace could not be executed here.
