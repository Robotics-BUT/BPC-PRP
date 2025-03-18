//
// Created by kuko on 17.3.25.
//
#include <gtest/gtest.h>
#include "../include/nodes/kinematics_odometry.hpp"
#include <cmath>

using namespace algorithms;

constexpr float ERROR = 0.001;
constexpr float WHEEL_BASE = 0.12834;
constexpr float WHEEL_RADIUS = 0.03288;
constexpr float WHEEL_CIRCUMFERENCE = 2 * M_PI * WHEEL_RADIUS;
constexpr int32_t PULSES_PER_ROTATION = 576;


TEST(KinematicsTest, BackwardZeroVelocitySI) {
    constexpr float linear = 0;
    constexpr float angular = 0;
    constexpr float expected_l = 0;
    constexpr float expected_r = 0;

    KinematicsOdometry kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.inverse(RobotSpeed {linear, angular});
    EXPECT_NEAR(result.l, expected_l, ERROR);
    EXPECT_NEAR(result.r, expected_r, ERROR);
}

TEST(KinematicsTest, BackwardPositiveLinearVelocitySI) {
    constexpr float linear = 1.0;
    constexpr float angular = 0;
    constexpr float expected_l = 1.0 / WHEEL_CIRCUMFERENCE * 2 * M_PI;
    constexpr float expected_r = 1.0 / WHEEL_CIRCUMFERENCE * 2 * M_PI;

    KinematicsOdometry kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.inverse(RobotSpeed {linear,angular});
    EXPECT_NEAR(result.l, expected_l, ERROR);
    EXPECT_NEAR(result.r, expected_r, ERROR);
}

TEST(KinematicsTest, BackwardPositiveAngularVelocitySI) {
    constexpr float linear = 1.0;
    constexpr float angular = 0;

    //tady ten test je podle me blbe
    constexpr float expected_l = -(0.5 * WHEEL_BASE) / WHEEL_CIRCUMFERENCE * (2 * M_PI);
    constexpr float expected_r = +(0.5 * WHEEL_BASE) / WHEEL_CIRCUMFERENCE * (2 * M_PI);

    KinematicsOdometry kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.inverse(RobotSpeed{linear, angular});
    EXPECT_NEAR(result.l, expected_l, ERROR);
    EXPECT_NEAR(result.r, expected_r, ERROR);
}

TEST(KinematicsTest, ForwardZeroWheelSpeedSI) {
    constexpr float wheel_l = 0;
    constexpr float wheel_r = 0;
    constexpr float expected_l = 0;
    constexpr float expected_a= 0;

    KinematicsOdometry kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.forward(WheelSpeed {wheel_l,wheel_r});
    EXPECT_NEAR(result.v, expected_l, ERROR);
    EXPECT_NEAR(result.w, expected_a, ERROR);
}

TEST(KinematicsTest, ForwardEqualWheelSpeedsSI) {
    constexpr float wheel_l = 1;
    constexpr float wheel_r = 1;
    constexpr float expected_l = WHEEL_RADIUS;
    constexpr float expected_a= 0;

    KinematicsOdometry kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.forward(WheelSpeed {wheel_l,wheel_r});
    EXPECT_NEAR(result.v, expected_l, ERROR);
    EXPECT_NEAR(result.w, expected_a, ERROR);
}

TEST(KinematicsTest, ForwardOppositeWheelSpeedsSI) {
    constexpr float wheel_l = -1;
    constexpr float wheel_r = 1;
    constexpr float expected_l = 0;
    constexpr float expected_a= (WHEEL_RADIUS / (0.5 * WHEEL_BASE));

    KinematicsOdometry kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.forward(WheelSpeed {wheel_l,wheel_r});
    EXPECT_NEAR(result.v, expected_l, ERROR);
    EXPECT_NEAR(result.w, expected_a, ERROR);;
}

TEST(KinematicsTest, ForwardAndBackwardSI) {
    constexpr float wheel_l = 1;
    constexpr float wheel_r = -0.5;

    KinematicsOdometry kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto lin_ang = kin.forward(WheelSpeed {wheel_l,wheel_r});
    auto result = kin.inverse(lin_ang);
    EXPECT_NEAR(result.l, wheel_l, ERROR);
    EXPECT_NEAR(result.r, wheel_r, ERROR);
}


TEST(KinematicsTest, ForwardAndBackwardEncoderDiff) {
    constexpr int encoder_l = 0;
    constexpr int encoder_r = 550;

    KinematicsOdometry kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto d_robot_pose = kin.forward(Encoders {encoder_l,encoder_r});
    auto result = kin.inverse(d_robot_pose);
    EXPECT_NEAR(result.l, encoder_l, 1);
    EXPECT_NEAR(result.r, encoder_r, 1);
}

// Main function to run all tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    std::cout<<"testy v cajku"<<std::endl;
    return RUN_ALL_TESTS();
}
