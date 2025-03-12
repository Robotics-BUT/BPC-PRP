#include "nodes/motor.hpp"
#include <chrono>
#include <iostream>

using namespace nodes;

MotorController::MotorController()
    : Node("motor_controller") {
    motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/set_motor_speeds", 10);
    encoder_subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
        "/bpc_prp_robot/encoders", 10, std::bind(&MotorController::encoder_callback, this, std::placeholders::_1));
}

void MotorController::set_motor_speeds(const WheelSpeed& speeds) {
    auto msg = std_msgs::msg::UInt8MultiArray();
    msg.data.resize(2);
    msg.data[0] = static_cast<uint8_t>(std::clamp(speeds.l, 0.0f, 255.0f));
    msg.data[1] = static_cast<uint8_t>(std::clamp(speeds.r, 0.0f, 255.0f));
    motor_publisher_->publish(msg);
}

Encoders MotorController::get_encoder_data() const {
    return encoders_;
}

void MotorController::encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
        encoders_.l = static_cast<int>(msg->data[0]);
        encoders_.r = static_cast<int>(msg->data[1]);
    }
}

