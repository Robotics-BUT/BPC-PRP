#include "nodes/button.hpp"

namespace nodes
{
    ButtonListener::ButtonListener() : Node("button_listener"), active_(false)
    {
        subscription_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/bpc_prp_robot/buttons", 10,
            std::bind(&ButtonListener::button_callback, this, std::placeholders::_1));
    }

    void ButtonListener::button_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        if (msg->data == 0)
        {
            active_ = !active_;
            RCLCPP_INFO(this->get_logger(), "Toggled state to: %s", active_ ? "ACTIVE" : "INACTIVE");
        }
    }

    bool ButtonListener::is_active() const
    {
        return active_;
    }
}


