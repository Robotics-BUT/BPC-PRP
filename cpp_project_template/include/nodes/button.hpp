#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace nodes
{
    class ButtonListener : public rclcpp::Node
    {
    public:
        ButtonListener();

        bool is_active() const;

    private:
        void button_callback(const std_msgs::msg::UInt8::SharedPtr msg);

        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;
        bool active_;
    };
}



