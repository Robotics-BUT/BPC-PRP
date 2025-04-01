// line_sensor_listener.hpp
#ifndef LINE_SENSOR_LISTENER_HPP
#define LINE_SENSOR_LISTENER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "motor.hpp"

namespace nodes {
    class LineSensorListener : public rclcpp::Node
    {
    public:
        LineSensorListener();
        void reset_integral();

    private:
        void on_line_sensors_msg(std::shared_ptr<std_msgs::msg::UInt16MultiArray> msg);

        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;
        std::shared_ptr<MotorController> motor_controller_;

        // PID
        float Kp;
        float Kd;
        float Ki;
        float base_speed;
        float last_error;
        float integral_;
    };
}
#endif // LINE_SENSOR_LISTENER_HPP



