#include "nodes/lidar.hpp"
#include <cmath>
#include <numeric>
#include <limits>

namespace nodes {

    LidarFiltrResults LidarFiltr::apply_filter(const std::vector<float>& points, float angle_start, float angle_end, float range_min, float range_max) {
        std::vector<float> front{}, back{}, left{}, right{};
        constexpr float PI = 3.14159265f;
        constexpr float angle_range = PI / 8.0f;

        float angle_step = (angle_end - angle_start) / points.size();

        float angle_offset=PI / 4.0f;
        for (size_t i = 0; i < points.size(); ++i) {
            float angle = angle_start + i * angle_step;
            float distance = points[i];

            if (!std::isfinite(distance) || distance <= 0.0f) continue;

            std::clamp(distance, range_min, range_max);

            while (angle > PI) angle -= 2 * PI;
            while (angle < -PI) angle += 2 * PI;

            if (std::abs(angle) <= angle_range) {
                front.push_back(distance);
            } else if (std::abs(angle - PI) <= angle_range || std::abs(angle + PI) <= angle_range) {
                back.push_back(distance);
            } else if (angle > angle_range +angle_offset && angle < PI - angle_range +angle_offset) {
                left.push_back(distance);
            } else if (angle < -angle_range- angle_offset&& angle > -PI + angle_range-  angle_offset) {
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
          Kp(0.5), Kd(2), Ki(0),
          base_speed(136),
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
        auto result = filter.apply_filter(msg->ranges, msg->angle_min, msg->angle_max, msg->range_min,msg->range_max);

        float error = result.right - result.left;
        //std::clamp(error, 5.0f, -5.0f);

        float error_avg = (error + last_error) / 2.0f;
        float d_error = error_avg - last_error;
        integral_ += error_avg;

        float integral_saturation=2;
        if (integral_ >  integral_saturation) integral_ =  integral_saturation;
        if (integral_ < -integral_saturation) integral_ = - integral_saturation;

        float speed_diff = Kp * error_avg + Kd * d_error + Ki * integral_;

        if (speed_diff > base_speed - 128) speed_diff = base_speed - 128;
        if (speed_diff < 128 - base_speed) speed_diff = 128 - base_speed;

        float left_motor_speed = base_speed - speed_diff;
        float right_motor_speed = base_speed + speed_diff;

        motor_controller_->set_motor_speeds({
            std::clamp(left_motor_speed, 128.0f, 2*base_speed-128),
            std::clamp(right_motor_speed, 128.0f, 2*base_speed-128)
        });

        last_error = error;

        RCLCPP_INFO(this->get_logger(), "L: %.2f R: %.2f Err: %.2f SpeedDiff: %.2f integral: %.2f",
                    result.left, result.right, error, speed_diff,integral_);
    }

}



