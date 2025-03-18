#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <vector>

struct RobotSpeed {
    float v; // linear velocity
    float w; // angular velocity
};

struct WheelSpeed {
    float l; // left wheel speed
    float r; // right wheel speed
};

struct Encoders {
    int l; // left wheel encoder value
    int r; // right wheel encoder value
};

struct Coordinates {
    float x;
    float y;
};

class Kinematics {
public:
    Kinematics(double wheel_radius, double wheel_base, int ticks_revolution);
    RobotSpeed forward(WheelSpeed x) const;
    WheelSpeed inverse(RobotSpeed x) const;
    Coordinates forward(Encoders x) const;
    Encoders inverse(Coordinates x) const;

private:
    double wheel_radius_;
    double wheel_base_;
    int ticks_revolution_;
};

namespace nodes {
    class MotorController : public rclcpp::Node {
    public:
        MotorController();
        ~MotorController() override = default;

        void set_motor_speeds(const WheelSpeed& speeds);

    private:
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_publisher_;
    };
}

