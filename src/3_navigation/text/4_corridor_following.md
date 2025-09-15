# Corridor Following

This guide explains how to regulate a differential chassis robot equipped with a 2D 360-degree LIDAR to follow a rectangular grid corridor.

## Background Concepts

### Differential Chassis
A differential chassis robot uses two independently controlled wheels to steer. By varying the speed of each wheel, the robot can:
 - Move Forward: Both wheels at the same speed.
 - Turn Left: Right wheel faster than the left.
 - Turn Right: Left wheel faster than the right.
 - Rotate in Place: Wheels move in opposite directions.

### 2D LIDAR
A 2D LIDAR scans the environment by emitting laser beams and measuring distances to objects. For a 360-degree LIDAR:
 - Distance Data: Provides distances to nearby obstacles in all directions.
 - Angle Data: Maps each distance reading to a specific angle.

### Rectangular Grid and Corridors
Corridors on a rectangular grid are linear paths with walls on either side. Key features:
 - Wall Alignment: Corridors are straight or have right-angle turns.
 - Center Line: The robot must maintain its position relative to the corridor’s center.
 - Obstacle Detection: Walls define the boundaries, and gaps or openings indicate intersections or exits.

## Corridor Following Algorithm

### Key Steps

 - LIDAR Data Processing: Analyze LIDAR scans to detect walls and the robot’s position relative to them.

 - Error Calculation: Determine the deviation from the corridor’s center line.

 - Control Response: Adjust wheel speeds to reduce the deviation.

### Wall Detection

  - Segment LIDAR Data: Divide the 360-degree scan into front, left, and right regions.
  - Identify Walls:
    - Use distance thresholds to detect walls.
    - (Optional) Fit linear equations to points to confirm wall alignment.
  - Calculate Midpoint: Determine the midpoint between the detected walls to establish the center line.

### Error Calculation

  - Define Error: The lateral distance between the robot’s position and the center line.
  - Angle Deviation: If wall orientation is estimated use it to estimate the angular alignment of the robot relative to the corridor.
  - Combined Error: A weighted sum of lateral and angular errors.

### Control Algorithm

Proportional-Derivative (PD) Control: Use proportional and derivative terms to regulate movement.

  - P-Term: Corrects based on the current error.
  - D-Term: Dampens oscillations by considering the rate of error change.

Control Formulas:
```
Left Motor Speed = Base Speed - (Kp * Error + Kd * Derivative of Error)
Right Motor Speed = Base Speed + (Kp * Error + Kd * Derivative of Error)
```

Obstacle Handling: Stop or adjust speed if a sudden obstacle is detected within a threshold distance.

## Example Pseudo Code

```python
import lidar_library
import motor_control

Kp = 0.5
Kd = 0.1
base_speed = 150  # actuator units (e.g., PWM)
last_error = 0.0
obstacle_threshold = 0.3  # meters

lidar = lidar_library.LIDAR()
motors = motor_control.MotorDriver()

def detect_walls(scan):
    left_distances = [dist for angle, dist in scan if 80 <= angle <= 100]
    right_distances = [dist for angle, dist in scan if 260 <= angle <= 280]
    left_wall = min(left_distances) if left_distances else None
    right_wall = min(right_distances) if right_distances else None
    return left_wall, right_wall

while True:
    scan = lidar.get_scan()
    left_wall, right_wall = detect_walls(scan)

    # Simple obstacle stop: if something is very close ahead
    front = [dist for angle, dist in scan if -10 <= angle <= 10 or 350 <= angle <= 360]
    if front and min(front) < obstacle_threshold:
        motors.set_speeds(0, 0)
        continue

    # Define error: positive if closer to right wall (robot needs to steer left)
    if left_wall is None and right_wall is None:
        # No walls detected: slow search
        motors.set_speeds(0, 0)
        continue
    elif left_wall is None:
        error = -(right_wall)  # steer left towards center
    elif right_wall is None:
        error = left_wall      # steer right towards center
    else:
        error = (left_wall - right_wall) / 2.0

    d_error = error - last_error

    left_speed = base_speed - (Kp * error + Kd * d_error)
    right_speed = base_speed + (Kp * error + Kd * d_error)

    # Constrain to valid actuator range
    left_speed = max(0, min(255, left_speed))
    right_speed = max(0, min(255, right_speed))

    motors.set_speeds(left_speed, right_speed)
    last_error = error
```

## Grid Pattern Following

If the corridor network is organized as a rectangular grid, the algorithm becomes more complex.

- View the space as a set of grid cells separated by walls or openings.
- If an opening appears on the left or right, temporarily rely on the existing wall and maintain heading until a new wall segment is detected.
- If there is an obstacle in front of the robot, stop and consider turning left or right based on your navigation policy (e.g., keep following the right wall).

A more advanced approach is to treat the environment as a discrete grid-cell map and control the robot during inter-cell transitions, using wall detections as events for state updates.



