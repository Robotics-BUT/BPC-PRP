#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

namespace nodes {

    struct LidarFiltrResults {
        float front;
        float back;
        float left;
        float right;
    };

    class LidarFiltr {
    public:
        LidarFiltrResults apply_filter(const std::vector<float>& points, float angle_start, float angle_end, float range_min, float range_max);
    };

    class LidarFilterNode : public rclcpp::Node {
    public:
        LidarFilterNode();

    private:
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    };

}
