#include "../include/nodes/kinematics_odometry.hpp"

namespace algorithms
{
KinematicsOdometry::KinematicsOdometry(double wheel_radius, double wheel_base, int ticks_revolution)
    : Node("kinematics_odometry"),
      wheel_radius_(wheel_radius),
      wheel_base_(wheel_base),
      ticks_revolution_(ticks_revolution),
      wheel_circumference_(2 * M_PI * wheel_radius) {

    encoder_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/bpc_prp_robot/encoders", 10,
        std::bind(&KinematicsOdometry::encoderCallback, this, std::placeholders::_1));
}

/**
    MotorController::MotorController()
        : Node("motor_controller") {
    motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/set_motor_speeds", 10);
    encoder_subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
        "/bpc_prp_robot/encoders", 10, std::bind(&MotorController::encoder_callback, this, std::placeholders::_1));
**/




void KinematicsOdometry::encoderCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
        updateEncoders(msg->data[0], msg->data[1]);
    }
}

void KinematicsOdometry::updateEncoders(int left, int right) {
    std::lock_guard<std::mutex> lock(encoder_mutex_);
    encoder_left_ = left;
    encoder_right_ = right;
    Coordinates new_position = forward(Encoders{left, right});
    theta_ += (static_cast<double>(right - left) / ticks_revolution_) * wheel_circumference_ / wheel_base_;
    position_ = new_position;
}

Coordinates KinematicsOdometry::getPosition() const {
    std::lock_guard<std::mutex> lock(encoder_mutex_);
    return position_;
}

RobotSpeed KinematicsOdometry::forward(WheelSpeed x) const {
    RobotSpeed result;
    result.v = (wheel_radius_ / 2.0) * (x.l + x.r);
    result.w = (x.r - x.l) * wheel_radius_ / wheel_base_;
    return result;
}

WheelSpeed KinematicsOdometry::inverse(RobotSpeed x) const {
    WheelSpeed result;
    result.l = (x.v - (x.w * (wheel_base_ / 2.0))) / wheel_radius_;
    result.r = (x.v + (x.w * (wheel_base_ / 2.0))) / wheel_radius_;
    return result;
}

Coordinates KinematicsOdometry::forward(Encoders x) const {
    Coordinates result;
    result.x = (static_cast<double>(x.l) / ticks_revolution_) * wheel_circumference_;
    result.y = (static_cast<double>(x.r) / ticks_revolution_) * wheel_circumference_;
    return result;
}

Encoders KinematicsOdometry::inverse(Coordinates x) const {
    Encoders result;
    result.l = static_cast<int>((x.x / wheel_circumference_) * ticks_revolution_);
    result.r = static_cast<int>((x.y / wheel_circumference_) * ticks_revolution_);
    return result;
}

} // namespace algorithms