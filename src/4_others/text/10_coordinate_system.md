# Coordinate System

Understanding coordinate systems is essential for robotics, computer vision, and navigation. Coordinates let us describe where things are and how they are oriented in space. This chapter summarizes the most useful concepts and conventions you will meet in practice.

## Frames, Axes, and Handedness

- Frame (coordinate frame): A local reference defined by an origin and three perpendicular axes (x, y, z). In 2D, you have two axes (x, y).
- Global vs. local: A global frame is a common reference for the whole scene (e.g., “map”). Local frames are attached to moving objects (e.g., “base_link” on a robot, “camera_link” on a camera).
- Handedness: The orientation of axes can be right‑handed or left‑handed. Most robotics software (ROS, linear algebra libraries) uses a right‑handed system.
  - Right‑hand rule: Point index finger along +x, middle finger along +y, then the thumb gives +z.

Why it matters: Mixing left‑ and right‑handed systems or unclear axis labels causes mirrored motions, flipped images, and hard‑to‑debug sign errors.

## 2D vs. 3D

- 2D pose: (x, y, θ), where θ is heading/yaw about z.
- 3D pose: position (x, y, z) + orientation (rotation in 3D).
- Units: Use meters for distances and radians for angles unless otherwise documented. Be explicit.

## Orientations in 3D

There are several equivalent ways to represent 3D rotation. Choose the one that fits your computations and interfaces.

- Euler angles (roll, pitch, yaw): Rotations about axes (commonly x, y, z). Easy to read; prone to gimbal lock and ambiguity (intrinsic vs. extrinsic, order matters: XYZ ≠ ZYX). Good for UI or logs, not for core math.
- Axis–angle: A unit axis vector â and an angle θ describe a single rotation about â. Compact and geometric.
- Rotation matrix R (3×3): Orthogonal matrix with det(R)=+1. Converts vectors between frames: v_B = R_AB · v_A (vector expressed in frame A to frame B). Great for composition; expensive to store many of them.
- Quaternion q = [w, x, y, z]: A unit quaternion encodes rotation without gimbal lock and composes efficiently. Preferred for interpolation and filtering.

Tip: Store orientation as a quaternion internally; convert to Euler only when needed for display.

## Homogeneous Coordinates and Transforms

To represent position and orientation together, we use a 4×4 homogeneous transform T:

- Form: T = [ R  t; 0  0  0  1 ], where R is 3×3 rotation and t is 3×1 translation.
- Applying a transform: In homogeneous coordinates p' = T · p, with p = [x, y, z, 1]^T.
- Composition: T_AC = T_AB · T_BC. Note the order: matrix multiplication is not commutative.
- Inverse: T_AB⁻¹ = [ Rᵀ  −Rᵀ t; 0 0 0 1 ]. Use it to “go back” from B to A.

Interpretation: T_AB “takes” coordinates of a point expressed in frame B and returns coordinates expressed in frame A.

## Composing and Chaining Frames (TF)

In robotics you often maintain a tree of frames (a “TF tree” in ROS terminology). Examples: map → odom → base_link → sensor frames.

- Composition rule: To express something in a different frame, multiply along the path in the correct order.
- Static vs. dynamic transforms: Some transforms are constant (sensor mounting), others change over time (robot motion). Time stamps matter when synchronizing sensors.
- Visualization: Tools like RViz2 can display axes for frames and arrows for transforms, helping you verify conventions early.

## Common Frames in Robotics (ROS conventions)

ROS REP‑103 defines standard frames and axis directions (right‑handed, meters, radians):

- map: A world‑fixed frame; global, not required to be continuous.
- odom: A world‑fixed frame with continuous, drift‑accumulating motion estimate (no jumps).
- base_link: The robot body frame; usually at the robot’s geometric center on the ground plane.
- camera_link / camera_optical_frame: Camera frames; optical frame typically has +z forward (optical axis), +x right, +y down in ROS.
- imu_link, laser, wheel frames, etc., attached as children of base_link.

Always document where the origin is and how axes are oriented for each sensor. This makes extrinsic calibration reproducible.

## Conventions and Best Practices

- Be explicit about units (m, rad) and handedness (right‑hand).
- Name frames consistently and publish a TF tree that matches your documentation.
- Keep transforms orthonormal: re‑normalize quaternions; ensure RᵀR ≈ I.
- Use consistent rotation order if using Euler angles; document it (e.g., XYZ intrinsic).
- Prefer quaternions for computation and interpolation; convert to Euler only for human‑readable outputs.
- Validate with visualization; a simple mistake (e.g., swapped axes) is obvious in RViz2.

## Typical Pitfalls

- Degrees vs. radians: Many libraries expect radians; mixing them leads to large errors.
- Gimbal lock with Euler angles: Avoid for continuous orientation tracking.
- Flipped camera axes: Image coordinates (u, v) vs. camera optical frames can differ; check your library’s convention.
- Frame direction confusion: Clarify whether T_AB maps B→A or A→B. Name accordingly (read subscripts left‑to‑right for mapping direction).
- Non‑commutativity: R1·R2 ≠ R2·R1. Keep multiplication order straight.

## Quick Cheat Sheet

- Right‑hand rule: x × y = z.
- Transform a point: p_A = T_AB · p_B.
- Compose transforms: T_AC = T_AB · T_BC.
- Inverse transform: T_BA = T_AB⁻¹ = [ Rᵀ  −Rᵀ t; 0 0 0 1 ].
- Quaternion normalization: q ← q / ||q|| before use.
- From yaw (2D): R(θ) = [[cosθ, −sinθ],[sinθ, cosθ]].

## Further Reading

- ROS REP‑103 (Standard Units of Measure and Coordinate Conventions): https://www.ros.org/reps/rep-0103.html
- A gentle quaternion primer (Eigen): https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html
- 3D Rotations, matrices, and quaternions (Wikipedia overview): https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions
- tf2 tutorials (ROS2): https://docs.ros.org/en/foxy/Tutorials/tf2.html
- RViz2 basics: see “9_rviz2_visualizations.md” in this repository.

