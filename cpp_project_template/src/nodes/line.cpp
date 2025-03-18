#include "nodes/line.hpp"

LineFollower::LineFollower() : Node("line_follower"), current_sensor_data_(std::make_shared<std_msgs::msg::UInt16MultiArray>()) {
    line_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
        "/bpc_prp_robot/line_sensors", 10,
        std::bind(&LineFollower::on_line_sensors_msg, this, std::placeholders::_1));
}

LineFollower::~LineFollower() {}


void LineFollower::on_line_sensors_msg(std::shared_ptr<std_msgs::msg::UInt16MultiArray> msg) {
    if (msg->data.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Invalid sensor data received.");
        return;
    }

    current_sensor_data_ = msg;  // Uložíme aktuálne dáta zo senzorov
    float left_value = static_cast<float>(msg->data[0]);
    float right_value = static_cast<float>(msg->data[1]);

    // Určujeme, ktorý senzor detekuje čiaru na základe prahu
    if (left_value > 512) {
        RCLCPP_INFO(this->get_logger(), "Left sensor detects the line.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Left sensor does NOT detect the line.");
    }

    if (right_value > 512) {
        RCLCPP_INFO(this->get_logger(), "Right sensor detects the line.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Right sensor does NOT detect the line.");
    }
}


std::shared_ptr<std_msgs::msg::UInt16MultiArray> LineFollower::get_line_sensor_data() const {
    return current_sensor_data_;  // Vrátime aktuálne dáta senzorov
}

// Implementácia metódy na získanie hodnoty DiscreteLinePose
DiscreteLinePose LineFollower::get_discrete_line_pose() const {

    float left_value = 0.0f;
    float right_value = 0.0f;

    if (left_value > 50 && right_value > 50) {
        return DiscreteLinePose::LineBoth;
    } else if (left_value > 50) {
        return DiscreteLinePose::LineOnLeft;
    } else if (right_value > 50) {
        return DiscreteLinePose::LineOnRight;
    } else {
        return DiscreteLinePose::LineNone;
    }
}

