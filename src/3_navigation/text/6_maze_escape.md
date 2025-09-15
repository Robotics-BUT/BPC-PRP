# Maze Escape

This short guide outlines a simple strategy for escaping a maze with a differential-drive robot using the right-hand rule (wall following). It is intentionally minimal and practical.

## Idea
- Keep your right side close to a wall and keep moving forward.
- When you hit an opening on the right, turn right and follow the new corridor.
- If you reach a dead end, turn around and continue following the right wall.

This works in simply-connected mazes (no isolated loops). In general mazes with loops, it still explores systematically but may revisit areas.

## Sensing Options
- Proximity sensors or bumpers (short range): detect walls and contacts.
- IR/ultrasonic rangefinders: measure distance to right/left/front walls.
- 2D LIDAR: robust wall detection with angles and distances.

## Minimal Right-Hand Rule (pseudocode)
```python
RIGHT = 1     # sensor index or side identifier
FRONT = 0

while True:
    right_clear = sense_opening(side=RIGHT)
    front_clear = sense_opening(side=FRONT)

    if right_clear:
        turn_right()
        drive_forward()
    elif front_clear:
        drive_forward()
    else:
        turn_left()  # or turn around if completely blocked
```

## Practical Tips
- Maintain a small, roughly constant right-wall distance if you have distance sensors; a simple PD controller on lateral error improves stability.
- Use a minimum forward speed to avoid stalling, and cap turn rates for smooth motion.
- Debounce sensor changes and use timeouts to avoid oscillations at junctions.

## Troubleshooting
- Robot gets stuck oscillating at corners: reduce speed, add a short delay after turns, and/or add hysteresis to “opening detected.”
- Loses the wall at gaps/doorways: keep moving forward briefly while searching for the wall again; if not found, slow down and rotate to reacquire.
- Drifts into walls: add a small proportional correction on measured right-wall distance.
