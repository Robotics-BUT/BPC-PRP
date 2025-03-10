# Lab 5 - Motor, Kinematics & Gamepad

Responsible: Ing. Jakub Minařík


## Tasks  

### 1. Motor Node Implementation  
- Develop a **motor node** that publishes wheel velocity commands to a ROS topic (`/bpc_prp_robot/set_motor_speeds`).  
- Ensure the node can send appropriate velocity commands to drive the robot’s wheels.


### 2. Encoder Node Implementation  
- Extend the **motor node**  or create separate **encoder node**to subscribe to an encoder topic for both wheels(`/bpc_prp_robot/encoders`).



### 3. Robot Parameter Estimation  
- Measure, estimate, or derive key robot parameters, such as:  
  - The relationship between commanded wheel velocity and actual wheel rotation speed.  
  - The relationship between wheel velocity, wheel radius, and chassis dimensions.  
  - The kinematic constraints affecting the robot’s movement.  
- Motor control values are represented as unsigned 8-bit integers (0–255):
  - A value of 127 corresponds to a neutral state (motors do not move).
  - Values greater than 127 cause the wheels to rotate forward.
  - Values less than 127 cause the wheels to rotate backward.
- The robot should execute the commanded speed for 1 second before stopping.
- Gearbox ration should be 1:48 and number of poles is propably 3 pairs of poles. Recomended to test if number of ticks makes full rotation of wheel.
- Test whether the number of encoder ticks corresponds to a full wheel rotation by counting the ticks per revolution.
- For additional information, refer to the motor datasheets and check the [robot's repository](https://github.com/Robotics-BUT/fenrir-project).


### 4. Kinematics and Odometry Computation  
- Implement a **class for kinematics and odometry calculations** for a differential drive robot.  
- Compute the robot's pose (position and orientation) based on wheel velocities and time.
- Implement dead reckoning using wheel encoders.  

### 5. Encoder Data Processing  
- Develop a **class for processing encoder data** or add to kinematics/odometry class:  
  - Estimate the robot’s displacement and position.  
  - Apply correction mechanisms using encoder feedback to improve localization accuracy.  

### 6. (Optional) Gamepad Control  
- Implement a **gamepad** node to manually control the robot’s movement.  
- Handle relevant gamepad events and publish speed for them.  

#### Instruction for Gamepad - SDL2
- Include SLD2 library `#include <SDL2/SDL.h>`
- Inicialize SDL2 - `SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER)`
- Find if joystick/gamepad is connected - `SDL_NumJoysticks()`
- Create gamepad object - `SDL_GameControllerOpen(0)`
- Poll events in time loop - made using ROS2 timer
  - Create event object - `SDL_Event`
  - Poll events - `SDL_PollEvent()`
  - check event types - e.g. `SDL_CONTROLLERBUTTONDOWN,SDL_CONTROLLERBUTTONUP,SDL_CONTROLLERAXISMOTION`
  - handle the events and set speed and rotation
  - publish ROS2 message
- Close gamepad object correctly - `SDL_GameControllerClose()`


## Tests Example
You can copy and create a test file from the example. You may also rename the Kinematics class and its methods or correct parameter types as needed.
```c++
#include <gtest/gtest.h>
#include "solution/algorithms/kinematics.hpp"
#include <cmath>

using namespace algorithms;

constexpr float ERROR = 0.001;
constexpr float WHEEL_BASE = 0.12;
constexpr float WHEEL_RADIUS = 0.033;
constexpr float WHEEL_CIRCUMFERENCE = 2 * M_PI * WHEEL_RADIUS;
constexpr int32_t PULSES_PER_ROTATION = 550;

TEST(KinematicsTest, BackwardZeroVelocitySI) {
    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.backward(Kinematics::LinearAngularSpeed{.linear=0.0, .angular=0.0});
    EXPECT_NEAR(result.left, 0.0, ERROR);
    EXPECT_NEAR(result.right, 0.0, ERROR);
}

TEST(KinematicsTest, BackwardPositiveLinearVelocitySI) {
    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.backward(Kinematics::LinearAngularSpeed{.linear=1.0, .angular=0.0});
    EXPECT_NEAR(result.left, 1.0 / WHEEL_CIRCUMFERENCE * 2 * M_PI, ERROR);
    EXPECT_NEAR(result.right, 1.0 / WHEEL_CIRCUMFERENCE * 2 * M_PI, ERROR);
}

TEST(KinematicsTest, BackwardPositiveAngularVelocitySI) {
    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.backward(Kinematics::LinearAngularSpeed{.linear=0.0, .angular=1.0});
    EXPECT_NEAR(result.left, -(0.5 * WHEEL_BASE) / WHEEL_CIRCUMFERENCE * (2 * M_PI), ERROR);
    EXPECT_NEAR(result.right, +(0.5 * WHEEL_BASE) / WHEEL_CIRCUMFERENCE * (2 * M_PI), ERROR);
}

TEST(KinematicsTest, ForwardZeroWheelSpeedSI) {
    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.forward(Kinematics::WheelsAngSpeed{.left=0.0, .right=0.0});
    EXPECT_NEAR(result.linear, 0.0, ERROR);
    EXPECT_NEAR(result.angular, 0.0, ERROR);
}

TEST(KinematicsTest, ForwardEqualWheelSpeedsSI) {
    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.forward(Kinematics::WheelsAngSpeed{.left=1.0, .right=1.0});
    EXPECT_NEAR(result.linear, WHEEL_RADIUS, ERROR);
    EXPECT_NEAR(result.angular, 0.0, ERROR);
}

TEST(KinematicsTest, ForwardOppositeWheelSpeedsSI) {
    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.forward(Kinematics::WheelsAngSpeed{.left=-1.0, .right=1.0});
    EXPECT_NEAR(result.linear, 0.0, ERROR);
    EXPECT_NEAR(result.angular, (WHEEL_RADIUS / (0.5 * WHEEL_BASE)), ERROR);
}

TEST(KinematicsTest, ForwardAndBackwardSI) {
    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto wheels = Kinematics::WheelsAngSpeed{.left=1.0, .right=-0.5};
    auto lin_ang = kin.forward(wheels);
    auto result = kin.backward(lin_ang);
    EXPECT_NEAR(result.left, wheels.left, ERROR);
    EXPECT_NEAR(result.right, wheels.right, ERROR);
}


TEST(KinematicsTest, ForwardAndBackwardEncoderDiff) {
    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto encoders_diff = Kinematics::EncoderDiff{.left=0, .right=550};
    auto d_robot_pose = kin.forward(encoders_diff);
    auto result = kin.backward(d_robot_pose);
    EXPECT_NEAR(result.left, encoders_diff.left, 1);
    EXPECT_NEAR(result.right, encoders_diff.right, 1);
}

// Main function to run all tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```
