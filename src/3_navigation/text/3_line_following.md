# Line Following

This guide explains how to regulate a differential chassis robot with two front-mounted line sensors to follow a line.

## Basic Concepts

### Differential Chassis

A differential chassis robot uses two independently controlled wheels to steer. Adjusting the speed of each wheel allows the robot to move forward, turn, or rotate in place. Key movements include:

Forward Movement: Both wheels move at the same speed.

Left Turn: The right wheel moves faster than the left.

Right Turn: The left wheel moves faster than the right.

### Line Sensors

Line sensors detect the contrast between a dark line and a lighter surface. Typically, two sensors are placed near the robot's front. Outputs:

Left Sensor (S1): Detects the line under the left side.

Right Sensor (S2): Detects the line under the right side.

Sensors usually output digital signals (1 for line detected, 0 for no line) or the analog signal (high voltage for line detected, low voltage for no line).

### Control Principles

Robots use control algorithms to maintain their position relative to the line. Key approaches:

Bang-Bang Control: Simple on/off control based on sensor inputs.

P(I)D Control: Smooth control using proportional and derivative terms based on sensor data.

## Line Following Algorithm

### Bang-Bang Control

Logic Table: Define responses based on sensor inputs:

| S1 | S2 | Action         |
|----|----|----------------|
| 1  | 1  | Move forward   |
| 1  | 0  | Turn left      |
| 0  | 1  | Turn right     |
| 0  | 0  | Stop or search |

Implementation:

If both sensors detect the line, drive both wheels forward.

If only the left sensor detects the line, slow down the left wheel and speed up the right wheel.

If only the right sensor detects the line, slow down the right wheel and speed up the left wheel.

If neither sensor detects the line, stop or initiate a search pattern.

### P(I)D Control

PD control improves performance by considering how far the robot deviates from the line and how fast the deviation changes.

Error Calculation:
 - Define error as the difference between sensor readings, e.g., ε = S₁ - S₂.
 - Use sensor output characteristics to determinate how far is S1 and S2 from the line center and estimate most probable position of the robot with respect to the line center.

Control Formula:
 - Adjust motor speeds using:
   - P-Term: Proportional to error (ε).
   - D-Term: Proportional to the rate of change of error (∆ε / Δt).
```
Left Motor Speed = Base Speed - (Kp * ε + Kd * ∆ε / Δt)
Right Motor Speed = Base Speed + (Kp * ε + Kd * ∆ε / Δt)
```

### Flowchart of the Algorithm
 - Read sensor values.
 - Calculate the error (∆ε) and its derivative (Δ∆ε / Δt).
 - Determine motor speeds using the control formula.
 - Drive the motors.
 - Repeat.

### Example Arduino Implementation

```c++
#define S1_PIN A0
#define S2_PIN A1
#define MOTOR_LEFT_PWM 3
#define MOTOR_RIGHT_PWM 5

float Kp = 0.5, Kd = 0.1;
float baseSpeed = 150;
float lastError = 0;

void setup() {
  pinMode(S1_PIN, INPUT);
  pinMode(S2_PIN, INPUT);
}

void loop() {
  int S1 = digitalRead(S1_PIN);
  int S2 = digitalRead(S2_PIN);

  float error = S1 - S2;
  float dError = error - lastError;

  float leftSpeed = baseSpeed - (Kp * error + Kd * dError);
  float rightSpeed = baseSpeed + (Kp * error + Kd * dError);

  analogWrite(MOTOR_LEFT_PWM, constrain(leftSpeed, 0, 255));
  analogWrite(MOTOR_RIGHT_PWM, constrain(rightSpeed, 0, 255));

  lastError = error;
}
```


## Testing and Calibration

Initial Test:
 - Run the robot on a simple track.
 - Observe behavior and ensure it detects and follows the line.

Tuning:
 - Adjust Kp to improve responsiveness.
 - Adjust Kd to reduce oscillations.

Advanced Testing:
 - Test on complex tracks with curves and intersections.
 - Optimize sensor placement for better detection.

## Troubleshooting
 - Robot veers off-line: Increase Kp.
 - Robot oscillates too much: Decrease Kd.
 - Robot fails to detect line: Ensure proper sensor calibration and placement.

## Extensions
 - Implement intersection handling.
 - Use more sensors for better precision.
 - Add PID control for further optimization.

