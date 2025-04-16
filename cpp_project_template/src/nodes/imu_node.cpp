//
// Created by kuko on 16.4.25.
//

#include "nodes/imu_node.hpp"
#include <numeric>

namespace nodes {

ImuNode::ImuNode() : Node("imu_node") {
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/bpc_prp_robot/imu",
        10,
        std::bind(&ImuNode::on_imu_msg, this, std::placeholders::_1)
    );

    planar_integrator_.reset();
    RCLCPP_INFO(this->get_logger(), "IMU Node initialized in INTEGRATE mode.");
}

void ImuNode::setMode(const ImuNodeMode setMode) {
    if (setMode == ImuNodeMode::CALIBRATE) {
        gyro_calibration_samples_.clear();
        planar_integrator_.reset();
        RCLCPP_INFO(this->get_logger(), "Switched to CALIBRATE mode.");
    } else if (setMode == ImuNodeMode::INTEGRATE) {
        if (!gyro_calibration_samples_.empty()) {
            planar_integrator_.setCalibration(gyro_calibration_samples_);
            RCLCPP_INFO(this->get_logger(), "Calibration complete. Switched to INTEGRATE mode.");
        } else {
            RCLCPP_WARN(this->get_logger(), "No calibration data. Proceeding without offset.");
        }
    }
    mode = setMode;
}

ImuNodeMode ImuNode::getMode() {
    return mode;
}

float ImuNode::getIntegratedResults() {
    return planar_integrator_.getYaw();  // radians
}

void ImuNode::reset_imu() {
    gyro_calibration_samples_.clear();
    planar_integrator_.reset();
    RCLCPP_INFO(this->get_logger(), "IMU integrator and calibration reset.");
}

void ImuNode::on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const float gyro_z = static_cast<float>(msg->angular_velocity.z);
    const rclcpp::Time stamp = msg->header.stamp;

    static rclcpp::Time last_stamp = stamp;

    if (mode == ImuNodeMode::CALIBRATE) {
        gyro_calibration_samples_.push_back(gyro_z);
    } else if (mode == ImuNodeMode::INTEGRATE) {
        double dt = (stamp - last_stamp).seconds();
        if (dt > 0.0) {
            planar_integrator_.update(gyro_z, dt);
        }
    }

    last_stamp = stamp;
}

void ImuNode::calibrate() {
    setMode(ImuNodeMode::CALIBRATE);
}

void ImuNode::integrate() {
    setMode(ImuNodeMode::INTEGRATE);
}

} // namespace nodes
