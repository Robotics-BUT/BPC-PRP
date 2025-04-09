#include "nodes/lidar.hpp"
#include <cmath>
#include <numeric>
#include <limits>

namespace nodes {

    LidarFiltrResults LidarFiltr::apply_filter(const std::vector<float>& points, float angle_start, float angle_end) {
        std::vector<float> front{}, back{}, left{}, right{};
        constexpr float PI = 3.14159265f;
        constexpr float angle_range = PI / 4.0f;

        float angle_step = (angle_end - angle_start) / points.size();

        for (size_t i = 0; i < points.size(); ++i) {
            float angle = angle_start + i * angle_step;
            float distance = points[i];

            if (!std::isfinite(distance) || distance <= 0.0f) continue;

            while (angle > PI) angle -= 2 * PI;
            while (angle < -PI) angle += 2 * PI;

            if (std::abs(angle) <= angle_range) {
                front.push_back(distance);
            } else if (std::abs(angle - PI) <= angle_range || std::abs(angle + PI) <= angle_range) {
                back.push_back(distance);
            } else if (angle > angle_range && angle < PI - angle_range) {
                left.push_back(distance);
            } else if (angle < -angle_range && angle > -PI + angle_range) {
                right.push_back(distance);
            }
        }

        auto average = [](const std::vector<float>& values) -> float {
            if (values.empty()) return std::numeric_limits<float>::quiet_NaN();
            return std::accumulate(values.begin(), values.end(), 0.0f) / values.size();
        };

        return LidarFiltrResults{
            .front = average(front),
            .back = average(back),
            .left = average(left),
            .right = average(right)
        };
    }

    LidarFilterNode::LidarFilterNode()
        : Node("lidar"),
          Kp(100.0), Kd(20.0), Ki(0.0),
          base_speed(140),
          last_error(0.0), integral_(0.0)
    {
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/bpc_prp_robot/lidar", 10,
            std::bind(&LidarFilterNode::scan_callback, this, std::placeholders::_1)
        );

        motor_controller_ = std::make_shared<MotorController>();
    }

    void LidarFilterNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        LidarFiltr filter;
        auto result = filter.apply_filter(msg->ranges, msg->angle_min, msg->angle_max);

        if (!std::isfinite(result.left) || !std::isfinite(result.right)) {
            motor_controller_->set_motor_speeds({0.0f, 0.0f});
            RCLCPP_WARN(this->get_logger(), "No valid lidar data.");
            return;
        }

        float error = result.right - result.left;
        float error_avg = (error + last_error) / 2.0f;
        float d_error = error_avg - last_error;
        integral_ += error_avg;

        if (integral_ > 2000) integral_ = 2000;
        if (integral_ < -2000) integral_ = -2000;

        float speed_diff = Kp * error_avg + Kd * d_error + Ki * integral_;

        if (speed_diff > base_speed - 128) speed_diff = base_speed - 128;
        if (speed_diff < 128 - base_speed) speed_diff = 128 - base_speed;

        float left_motor_speed = base_speed - speed_diff;
        float right_motor_speed = base_speed + speed_diff;

        motor_controller_->set_motor_speeds({
            std::clamp(left_motor_speed, 0.0f, 255.0f),
            std::clamp(right_motor_speed, 0.0f, 255.0f)
        });

        last_error = error;

        RCLCPP_INFO(this->get_logger(), "L: %.2f R: %.2f Err: %.2f SpeedDiff: %.2f",
                    result.left, result.right, error, speed_diff);
    }

}



