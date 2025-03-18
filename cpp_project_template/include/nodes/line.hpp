// line_follower.hpp
#ifndef LINE_FOLLOWER_HPP
#define LINE_FOLLOWER_HPP

#include <array>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

enum class DiscreteLinePose {
    LineOnLeft,
    LineOnRight,
    LineNone,
    LineBoth,
};

class LineFollower : public rclcpp::Node {
public:
    LineFollower();
    ~LineFollower();

    float get_continuous_line_pose() const;
    DiscreteLinePose get_discrete_line_pose() const;
    std::shared_ptr<std_msgs::msg::UInt16MultiArray> get_line_sensor_data() const;

private:
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;
    std::shared_ptr<std_msgs::msg::UInt16MultiArray> current_sensor_data_;

    void on_line_sensors_msg(std::shared_ptr<std_msgs::msg::UInt16MultiArray> msg);
    float estimate_continuous_line_pose(float left_value, float right_value);
    DiscreteLinePose estimate_discrete_line_pose(float l_norm, float r_norm);
};

#endif // LINE_FOLLOWER_HPP

