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


## Rechecked November 2, 2026 (UTC)

- `python3 -m pytest -v ros_uav_apriltag/tests/test_control_utils.py`: **3 collected tests passed**. The existing utility assertions are now named pytest cases and remain runnable through the original script entry point.
- `PYTHONPATH=ros_uav_apriltag python3 ros_uav_apriltag/tests/run_offline_validation.py`: **PASS**, 8 search waypoints, detection step 40, centered step 93, final XY error 0.184 m. The runner now asserts that HOVER is reached and the final error is below tolerance before reporting success.
- All package Python files parsed successfully and `package.xml` parsed successfully.

These checks ran without a camera, VIO estimator, flight controller or PX4 simulator. The earlier runtime limitation remains unchanged.
