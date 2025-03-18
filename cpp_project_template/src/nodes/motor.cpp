#include "nodes/motor.hpp"
#include <chrono>
#include <iostream>

using namespace nodes;

MotorController::MotorController()
    : Node("motor_controller") {
    motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/set_motor_speeds", 10);
}

void MotorController::set_motor_speeds(const WheelSpeed& speeds) {
    auto msg = std_msgs::msg::UInt8MultiArray();
    msg.data.resize(2);
    msg.data[0] = static_cast<uint8_t>(std::clamp(speeds.l, 0.0f, 255.0f));
    msg.data[1] = static_cast<uint8_t>(std::clamp(speeds.r, 0.0f, 255.0f));
    motor_publisher_->publish(msg);
}

