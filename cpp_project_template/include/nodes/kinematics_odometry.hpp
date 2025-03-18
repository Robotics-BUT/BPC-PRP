#ifndef KINEMATICS_ODOMETRY_HPP
#define KINEMATICS_ODOMETRY_HPP

#include <cmath>
#include <cstdint>
#include <atomic>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

namespace algorithms
{
    struct RobotSpeed {
        float v; // Linear velocity [m/s]
        float w; // Angular velocity [rad/s]
    };

    struct WheelSpeed {
        float l; // Left wheel speed [rad/s]
        float r; // Right wheel speed [rad/s]
    };

    struct Encoders {
        int l; // Left wheel encoder ticks
        int r; // Right wheel encoder ticks
    };

    struct Coordinates {
        float x; // Cartesian x-coordinate
        float y; // Cartesian y-coordinate
    };

    class KinematicsOdometry : public rclcpp::Node {
    public:
        KinematicsOdometry(double wheel_radius, double wheel_base, int ticks_revolution);

        RobotSpeed forward(WheelSpeed x) const;
        WheelSpeed inverse(RobotSpeed x) const;
        Coordinates forward(Encoders x) const;
        Encoders inverse(Coordinates x) const;

        void updateEncoders(int left, int right);
        Coordinates getPosition() const;

    private:
        void encoderCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

        double wheel_radius_;
        double wheel_base_;
        int ticks_revolution_;
        double wheel_circumference_;

        mutable std::mutex encoder_mutex_;
        std::atomic<int> encoder_left_{0};
        std::atomic<int> encoder_right_{0};
        Coordinates position_{0.0, 0.0};
        double theta_{0.0}; // Orientation angle of the robot

        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_sub_;
    };
}
#endif // KINEMATICS_ODOMETRY_HPP