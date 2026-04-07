from typing import List, Tuple


def generate_lawnmower(bounds: Tuple[float, float, float, float], lane_spacing: float, altitude: float) -> List[Tuple[float, float, float]]:
    """Generate lawnmower waypoints in ENU map frame.

    bounds: (xmin, xmax, ymin, ymax)
    """
    xmin, xmax, ymin, ymax = bounds
    if lane_spacing <= 0.0:
        raise ValueError('lane_spacing must be > 0')

    y = ymin
    direction = 1
    waypoints = []
    while y <= ymax + 1e-6:
        if direction > 0:
            waypoints.append((xmin, y, altitude))
            waypoints.append((xmax, y, altitude))
        else:
            waypoints.append((xmax, y, altitude))
            waypoints.append((xmin, y, altitude))
        direction *= -1
        y += lane_spacing
    return waypoints


def compute_xy_velocity(error_x: float, error_y: float, kp: float, max_vel: float) -> Tuple[float, float]:
    vx = kp * error_x
    vy = kp * error_y
    vx = max(-max_vel, min(max_vel, vx))
    vy = max(-max_vel, min(max_vel, vy))
    return vx, vy
