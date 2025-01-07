# Differential Chassis

The differential drive chassis is one of the most common configurations for mobile robots. It consists of two independently controlled wheels mounted on the same axis and optionally a passive caster wheel for stability.

This tutorial introduces the fundamental concepts of differential drive kinematics and demonstrates how to derive the forward and inverse kinematics equations.

## Components of a Differential Drive Chassis

 - Two wheels: Independently driven, providing linear and rotational motion.
 - Chassis: Holds the wheels, motors, and sensors.
 - Center of the robot: Defined as the midpoint between the two wheels.
 - Wheel radius (`r`): Radius of each wheel.
 - Wheel separation (`L`): Distance between the two wheels.

## Kinematic Model

Pose `(x,y,θ)`: The robot's position `(x,y)` and orientation `θ` in a 2D plane. `[m, m, rad]`

Linear velocity `(v)`: Forward speed of the robot. `[m/s]`

Angular velocity `(ω)`: Rate of rotation of the robot. `[rad/s]`

## Wheel Velocities

Left wheel angular velocity: `ωL`
Right wheel angular velocity: `ωR`

The linear velocities of the wheels are:

<p><img src="../images/dif_chass_vl_vr.png" alt="pid equation"/></p>

## Forward Kinematics

Forward kinematics calculates the robot's linear and angular velocities based on wheel velocities.

### Linear and Angular Velocities

The robot's linear velocity `(v)` and angular velocity `(ω)` are:

<p><img src="../images/dif_chass_v_omega.png" alt="forward kinematics"/></p>

### Pose Update

Given the robot's current pose `(x,y,θ)`, the new pose after a small time step `dt` can be computed as:

<p><img src="../images/dif_chass_update.png" alt="pose update"/></p>

##Inverse Kinematics

Inverse kinematics computes the wheel velocities required to achieve a desired linear and angular velocity.

Given:
 - Desired linear velocity `v`.
 - Desired angular velocity `ω`.

The wheel velocities are:

<p><img src="../images/dif_chass_inverse_v.png" alt="inverse kinematics, wheel speed"/></p>

To compute angular velocities:

<p><img src="../images/dif_chass_inverse_omega.png" alt="inverse kinematics, wheel speed"/></p>

## Example Code

```python
def forward_kinematics(v_L, v_R, L):
    v = (v_R + v_L) / 2
    omega = (v_R - v_L) / L
    return v, omega
```

```python
def update_pose(x, y, theta, v, omega, dt):
    x_new = x + v * math.cos(theta) * dt
    y_new = y + v * math.sin(theta) * dt
    theta_new = theta + omega * dt
    return x_new, y_new, theta_new
```

```python
def inverse_kinematics(v, omega, L):
    v_L = v - (omega * L / 2)
    v_R = v + (omega * L / 2)
    return v_L, v_R
```

## Exercise

Write program which simulates differential chassis based on given input parameters.

### Simulation Parameters

 - Wheel radius: `r = 0.1 m`
 - Wheel separation: `L = 0.15 m`
 - Time step: `dt = 0.01 s`

### Tasks

 - Compute the pose of the robot after moving straight for 5 seconds with `v = 1 m/s`.
 - Simulate a circular motion with `v = 1 m/s` and `ω = 0.5 rad/s` .
 - Simulate a circular motion with `ωl = 1 m/s` and `ωr = 0.5 rad/s` .