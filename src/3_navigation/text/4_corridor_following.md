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
base_speed = 150
last_error = 0

lidar = lidar_library.LIDAR()
motors = motor_control.MotorDriver()

def detect_walls(scan):
    left_distances = [dist for angle, dist in scan if 80 <= angle <= 100]
    right_distances = [dist for angle, dist in scan if 260 <= angle <= 280]
    left_wall = min(left_distances)
    right_wall = min(right_distances)
    return left_wall, right_wall

while True:
    scan = lidar.get_scan()
    left_wall, right_wall = detect_walls(scan)

    error = (left_wall - right_wall) / 2
    d_error = error - last_error

    left_speed = base_speed - (Kp * error + Kd * d_error)
    right_speed = base_speed + (Kp * error + Kd * d_error)

    motors.set_speeds(left_speed, right_speed)
    last_error = error
```

## Grid Pattern Following

If the corridor is organized into rectangular grid. The algorithm is getting more complex.

Think about the corridor as a set of cells that contains spaces or obstacles between each other.

If the empty space appears on left or right, the algorithm have to overcome this difficulty and consider only existing walls.

If there is an obstacle in front of the robot, stop and consider turing robot left or right.

More advanced approach is to think about the grid corridor not as a continuous space, but rather discrete grid-cell space. Than the corridor following algorithm have to control robot during the inter-cell transitions. 



