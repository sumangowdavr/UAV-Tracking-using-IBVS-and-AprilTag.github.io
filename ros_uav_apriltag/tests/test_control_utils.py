import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros_uav_apriltag.control_utils import generate_lawnmower, compute_xy_velocity


def test_lawnmower_waypoints():
    wps = generate_lawnmower((0.0, 4.0, 0.0, 4.0), lane_spacing=2.0, altitude=3.0)
    assert len(wps) == 6, f'unexpected waypoint count: {len(wps)}'
    assert wps[0] == (0.0, 0.0, 3.0)
    assert wps[1] == (4.0, 0.0, 3.0)
    assert wps[2] == (4.0, 2.0, 3.0)


def test_velocity_saturation():
    vx, vy = compute_xy_velocity(10.0, -10.0, kp=0.5, max_vel=1.0)
    assert vx == 1.0 and vy == -1.0


def test_proportional_velocity():
    vx2, vy2 = compute_xy_velocity(0.2, -0.2, kp=0.5, max_vel=1.0)
    assert abs(vx2 - 0.1) < 1e-9 and abs(vy2 + 0.1) < 1e-9

def main():
    test_lawnmower_waypoints()
    test_velocity_saturation()
    test_proportional_velocity()
    print('test_control_utils: PASS')


if __name__ == '__main__':
    main()
