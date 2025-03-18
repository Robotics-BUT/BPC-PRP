#include "nodes/motor.hpp"
#include "nodes/kinematics_odometry.hpp"
#include <thread>
#include <chrono>
#include <iostream>

using namespace nodes;
using namespace algorithms;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    double wheel_radius = 0.03288;
    double wheel_base = 0.12834;
    int ticks_revolution = 576;

    auto motor_controller = std::make_shared<MotorController>();
    auto kinematics_odometry = std::make_shared<KinematicsOdometry>(wheel_radius, wheel_base, ticks_revolution);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(motor_controller);
    executor->add_node(kinematics_odometry);

    std::thread executor_thread([executor]() { executor->spin(); });

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::vector<uint8_t> motor_speeds = {0, 0};
    motor_speeds[0] = 170;
    motor_speeds[1] = 165;

    motor_controller->set_motor_speeds({static_cast<float>(motor_speeds[0]), static_cast<float>(motor_speeds[1])});

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    algorithms::Coordinates position = kinematics_odometry->getPosition();

    std::cout << "Current Position: X = " << position.x << ", Y = " << position.y << std::endl;

    rclcpp::shutdown();
    executor_thread.join();

    return 0;
}
