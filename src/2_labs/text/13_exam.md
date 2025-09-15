# Final Exam - Maze Escape

Responsible: Ing. Adam Ligocki, Ph.D.

The final exam is a competition. Each team has up to 3 attempts to escape the maze using the robot’s sensors. The final score is based on the best attempt.
- The robot starts in the center of the starting cell.
- There is one optimal escape path.
- The maze contains no loops.
- There are 3 randomly placed minotaurs and 1 treasure.
  - Each minotaur encounter adds a 30 s penalty.
  - Finding the treasure subtracts 30 s from the final time.
- The maze consists of 8 × 8 cells; each cell is 0.40 × 0.40 m. Black tape on the floor marks the boundaries between cells.
- ArUco tags are placed on the floor and provide hints about the escape route, minotaurs, or treasure.

## Scoring

In total, a team can earn up to 50 points:
- Maze escape: up to 40 points.
  - Score is calculated as `y = min(max(kx + q, 0), 40)`.
- Git and project documentation quality: up to 10 points.

## Attempt rules

- Teams have at least 45 minutes between attempts to modify their program.
- The competition code must be uploaded to Git by 23:59 on the same day.
- The competition code must not contain specific information about the maze (e.g., paths, minotaur locations). Pre‑known constants (e.g., cell size) may be included.
- Do not touch the walls.

## Maze example
- Walls: red lines
- Escape path: green line
- Start: green cell
- Treasure (bonus): blue cell
- Minotaur (penalty): red cell

![Maze example with walls, path, start, treasure, and minotaurs](../images/maze.png)
