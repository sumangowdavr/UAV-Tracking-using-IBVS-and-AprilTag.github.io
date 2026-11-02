from ros_uav_apriltag.control_utils import generate_lawnmower, compute_xy_velocity


def run():
    dt = 0.1
    kp = 0.7
    max_vel = 0.8
    center_tol = 0.2

    waypoints = generate_lawnmower((0.0, 6.0, 0.0, 6.0), 2.0, 3.0)

    drone_x, drone_y = 0.0, 0.0
    tag_x, tag_y = 4.2, 3.1

    phase = 'SEARCH'
    detect_step = None
    centered_step = None

    for step in range(1, 301):
        if phase == 'SEARCH':
            # synthetic detect event after some search time
            if step == 40:
                phase = 'TRACK'
                detect_step = step
            else:
                # pseudo-search motion toward waypoints
                wp = waypoints[(step // 10) % len(waypoints)]
                err_x = wp[0] - drone_x
                err_y = wp[1] - drone_y
                vx, vy = compute_xy_velocity(err_x, err_y, 0.4, 0.6)
                drone_x += vx * dt
                drone_y += vy * dt
                continue

        if phase == 'TRACK':
            err_x = tag_x - drone_x
            err_y = tag_y - drone_y
            vx, vy = compute_xy_velocity(err_x, err_y, kp, max_vel)
            drone_x += vx * dt
            drone_y += vy * dt

            if (err_x ** 2 + err_y ** 2) ** 0.5 < center_tol:
                centered_step = step
                phase = 'HOVER'
                break

    final_err = ((tag_x - drone_x) ** 2 + (tag_y - drone_y) ** 2) ** 0.5

    assert phase == 'HOVER', f'mission ended in {phase}'
    assert detect_step is not None and centered_step is not None
    assert final_err < center_tol, f'tracking error {final_err} exceeds tolerance'

    print('offline_validation: PASS')
    print(f'waypoint_count={len(waypoints)}')
    print(f'detect_step={detect_step}')
    print(f'centered_step={centered_step}')
    print(f'final_xy_error_m={final_err:.3f}')


if __name__ == '__main__':
    run()
