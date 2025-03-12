#include "nodes/io_node.hpp"

namespace nodes {
    IoNode::IoNode() : Node("io_node") {
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/bpc_prp_robot/buttons", 10, std::bind(&IoNode::on_button_callback, this, std::placeholders::_1));
        led_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/rgb_leds", 10);
    }

    int IoNode::get_button_pressed() const {
        return button_pressed_;
    }

    void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        button_pressed_ = static_cast<int>(msg->data);
        RCLCPP_INFO(this->get_logger(), "Button pressed: %d", button_pressed_);
    }

    void IoNode::publish_leds(const std::vector<uint8_t>& colors) {
        std_msgs::msg::UInt8MultiArray msg;
        msg.data = colors;
        led_publisher_->publish(msg);
    }
}

// KINEMATICS/ODOMETRY
// dve funkce na uhlovou a linearni rychlost z rychlosti kol
