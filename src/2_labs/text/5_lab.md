# Lab 4 - Motor, Kinematics & Gamepad

Responsible: Ing. Jakub Minařík

## Tasks
The end result of this lab should be an estimate of position in Cartesian coordinates with the origin at the start position after driving the robot.

### 1. Motor publisher implementation
- Develop a motor node that publishes wheel velocity commands to a ROS 2 topic (`/bpc_prp_robot/set_motor_speeds`).
- Ensure the node can send appropriate velocity commands to drive the robot’s wheels.

### 2. Encoder subscriber implementation
- Extend the motor node or create a separate encoder node to subscribe to an encoder topic for both wheels (`/bpc_prp_robot/encoders`).

### 3. Robot parameter estimation
- Measure, estimate, or derive key robot parameters, such as:
  - The relationship between commanded wheel velocity and actual wheel rotation speed.
  - The relationship between wheel velocity, wheel radius, and chassis dimensions.
  - The kinematic constraints affecting the robot’s movement.
- Motor control values are represented as unsigned 8-bit integers (0–255):
  - A value of 127 corresponds to a neutral state (motors do not move).
  - Values greater than 127 cause the wheels to rotate forward.
  - Values less than 127 cause the wheels to rotate backward.
- The robot should execute the commanded speed for 1 second before stopping.
- The gearbox ratio is 1:48 and the motor likely has 3 pole pairs. It is recommended to test whether the number of ticks corresponds to one full wheel rotation.
- Test whether the number of encoder ticks corresponds to a full wheel rotation by counting the ticks per revolution.
- For additional information, refer to the motor datasheets and check the robot repository: https://github.com/Robotics-BUT/fenrir-project

### 4. Kinematics and odometry computation
- Implement a class for kinematics and odometry calculations for a differential drive robot.
- Compute the robot pose (position and orientation) based on wheel velocities and time.
- Implement dead reckoning using wheel encoders.

### 5. Encoder data processing
- Develop a class for processing encoder data (or add to the kinematics/odometry class):
  - Estimate the robot displacement and position.
  - Apply correction mechanisms using encoder feedback to improve localization accuracy.

### 6. (Optional) Gamepad control
- Implement a gamepad node to manually control the robot movement.
- Handle relevant gamepad events and publish speeds for them.

#### Instructions for gamepad — SDL2
- Include SDL2: `#include <SDL2/SDL.h>`
- Initialize SDL2: `SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER)`
- Check if a joystick/gamepad is connected: `SDL_NumJoysticks()`
- Create a gamepad object: `SDL_GameControllerOpen(0)`
- Poll events in a time loop (e.g., via a ROS 2 timer):
  - Create an event object: `SDL_Event`
  - Poll events: `SDL_PollEvent()`
  - Check event types, e.g., `SDL_CONTROLLERBUTTONDOWN`, `SDL_CONTROLLERBUTTONUP`, `SDL_CONTROLLERAXISMOTION`
  - Handle events and set speed and rotation
  - Publish a ROS 2 message
- Close the gamepad object: `SDL_GameControllerClose()`

## Tests example
You can copy and create a test file from the example. You will probably need to rename the Kinematics class and its methods or correct parameter types as needed.
```c++
#include <gtest/gtest.h>
#include "../include/kinematics.hpp"
#include <cmath>

using namespace algorithms;

constexpr float ERROR = 0.001f;
constexpr float WHEEL_BASE = 0.12f;
constexpr float WHEEL_RADIUS = 0.033f;
constexpr float WHEEL_CIRCUMFERENCE = 2 * M_PI * WHEEL_RADIUS;
constexpr int32_t PULSES_PER_ROTATION = 550;

TEST(KinematicsTest, BackwardZeroVelocitySI) {
    constexpr float linear = 0.0f;
    constexpr float angular = 0.0f;
    constexpr float expected_l = 0.0f;
    constexpr float expected_r = 0.0f;

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.inverse(RobotSpeed{linear, angular});
    EXPECT_NEAR(result.l, expected_l, ERROR);
    EXPECT_NEAR(result.r, expected_r, ERROR);
}

TEST(KinematicsTest, BackwardPositiveLinearVelocitySI) {
    constexpr float linear = 1.0f;
    constexpr float angular = 0.0f;
    constexpr float expected_l = 1.0f / WHEEL_CIRCUMFERENCE * 2 * M_PI;
    constexpr float expected_r = 1.0f / WHEEL_CIRCUMFERENCE * 2 * M_PI;

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.inverse(RobotSpeed{linear, angular});
    EXPECT_NEAR(result.l, expected_l, ERROR);
    EXPECT_NEAR(result.r, expected_r, ERROR);
}

TEST(KinematicsTest, BackwardPositiveAngularVelocitySI) {
    constexpr float linear = 1.0f;
    constexpr float angular = 0.0f;
    constexpr float expected_l = -(0.5f * WHEEL_BASE) / WHEEL_CIRCUMFERENCE * (2 * M_PI);
    constexpr float expected_r = +(0.5f * WHEEL_BASE) / WHEEL_CIRCUMFERENCE * (2 * M_PI);

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.inverse(RobotSpeed{linear, angular});
    EXPECT_NEAR(result.l, expected_l, ERROR);
    EXPECT_NEAR(result.r, expected_r, ERROR);
}

TEST(KinematicsTest, ForwardZeroWheelSpeedSI) {
    constexpr float wheel_l = 0;
    constexpr float wheel_r = 0;
    constexpr float expected_l = 0;
    constexpr float expected_a= 0;

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.forward(WheelSpeed {wheel_l,wheel_r});
    EXPECT_NEAR(result.v, expected_l, ERROR);
    EXPECT_NEAR(result.w, expected_a, ERROR);
}

TEST(KinematicsTest, ForwardEqualWheelSpeedsSI) {
    constexpr float wheel_l = 1;
    constexpr float wheel_r = 1;
    constexpr float expected_l = WHEEL_RADIUS;
    constexpr float expected_a= 0;

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.forward(WheelSpeed {wheel_l,wheel_r});
    EXPECT_NEAR(result.v, expected_l, ERROR);
    EXPECT_NEAR(result.w, expected_a, ERROR);
}

TEST(KinematicsTest, ForwardOppositeWheelSpeedsSI) {
    constexpr float wheel_l = -1;
    constexpr float wheel_r = 1;
    constexpr float expected_l = 0;
    constexpr float expected_a= (WHEEL_RADIUS / (0.5 * WHEEL_BASE));

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.forward(WheelSpeed {wheel_l,wheel_r});
    EXPECT_NEAR(result.v, expected_l, ERROR);
    EXPECT_NEAR(result.w, expected_a, ERROR);;
}

TEST(KinematicsTest, ForwardAndBackwardSI) {
    constexpr float wheel_l = 1;
    constexpr float wheel_r = -0.5;

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto lin_ang = kin.forward(WheelSpeed {wheel_l,wheel_r});
    auto result = kin.inverse(lin_ang);
    EXPECT_NEAR(result.l, wheel_l, ERROR);
    EXPECT_NEAR(result.r, wheel_r, ERROR);
}


TEST(KinematicsTest, ForwardAndBackwardEncoderDiff) {
    constexpr int encoder_l = 0;
    constexpr int encoder_r = 550;

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto d_robot_pose = kin.forward(Encoders {encoder_l,encoder_r});
    auto result = kin.inverse(d_robot_pose);
    EXPECT_NEAR(result.l, encoder_l, 1);
    EXPECT_NEAR(result.r, encoder_r, 1);
}

// Main function to run all tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

## Kinematics Header Example
Only example of header - types need to be corrected. Instead of structures you can use, for example `std::pair`. Function working with coordinates is working with differences.

```c++
 struct RobotSpeed{
  float v; //linear
  float w; //angluar
 }

 struct WheelSpeed{ //depends on you in what units
  float l; //left
  float r; //right
 }

struct Encoders{
  int l; //left
  int r; //right
}
Coordinates{ //Cartesian coordinates
  float x; 
  float y;
}

class Kinematics{
  Kinematics(double wheel_radius, double wheel_base, int ticks_revolution);
  RobotSpeed forward(WheelSpeed x) const;
  WheelSpeed inverse(RobotSpeed x) const;
  Coordinates forward(Encoders x) const;
  Encoders inverse(Coordinates x) const;
}
```

## Be Aware of Parallel Programming
When a variable is accessed by multiple threads—such as in the case of an encoder node, where a callback writes the encoder’s value to a variable while another thread reads it—you must use `std::mutex` or `std::atomic` to ensure thread safety. More about parallel computing in [Multithreading](../../4_others/text/8_multithreading.md).

### Atomic variables
Atomic variables are thread save, but only simple types such as int, float. 
Name is from atomic operation/instruction - this type of instruction cannot be interrupted when executed, so it blocks the memory until done and other threads are waiting.
```c++
std::atomic<int> atomic_int;
```
### Mutex
Mutex can be used to safely modify complex data structures such as `std::vector` or `std::map`.
A mutex works by locking a resource when a thread accesses it and unlocking it after the operation is complete. Other threads attempting to access the resource must wait until the mutex is released.
```c++
std::mutex mtx;
int shared_value = 0;

void increment()
{
    std::lock_guard<std::mutex> lock(mtx); 
    shared_value++;
}
```

