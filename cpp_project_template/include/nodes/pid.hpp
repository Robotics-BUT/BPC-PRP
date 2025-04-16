#ifndef PID_H
#define PID_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>
#include "nodes/lidar.hpp"
#include "nodes/motor.hpp"
#include "nodes/imu_node.hpp"

namespace nodes {

    enum class DriveState {
        DRIVE_FORWARD,
        TURNING
    };

    class PidNode : public rclcpp::Node {
    public:
        PidNode(std::shared_ptr<ImuNode> imu_node);

    private:
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        float Kp, Kd, Ki;
        float base_speed;
        float last_error;
        float integral_;

        DriveState state_;
        int turn_direction_; // -1 = doprava, 1 = doľava
        float turn_start_yaw_;

        std::shared_ptr<ImuNode> imu_node_;
        std::shared_ptr<MotorController> motor_controller_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    };


}

#endif // PID_H

