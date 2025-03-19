// line_sensor_listener.cpp
#include "nodes/line.hpp"

using namespace nodes;

LineSensorListener::LineSensorListener() : Node("line_sensor_listener"), Kp(0.01), Kd(0.005), base_speed(130), last_error(0.0)
{
    line_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
        "/bpc_prp_robot/line_sensors", 10,
        std::bind(&LineSensorListener::on_line_sensors_msg, this, std::placeholders::_1));

    motor_controller_ = std::make_shared<MotorController>();
}

void LineSensorListener::on_line_sensors_msg(std::shared_ptr<std_msgs::msg::UInt16MultiArray> msg)
{
    if (msg->data.size() >= 2)
    {
        float left_value = static_cast<float>(msg->data[0]);
        float right_value = static_cast<float>(msg->data[1]);

        RCLCPP_INFO(this->get_logger(), "Left Sensor: %f, Right Sensor: %f", left_value, right_value);

        float error = left_value - right_value;
        float d_error = error - last_error;

        float left_motor_speed = base_speed - (Kp * error + Kd * d_error);
        float right_motor_speed = base_speed + (Kp * error + Kd * d_error);

        motor_controller_->set_motor_speeds({
            std::clamp(left_motor_speed, 0.0f, 255.0f),
            std::clamp(right_motor_speed, 0.0f, 255.0f)
        });

        last_error = error;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Invalid sensor data received.");
    }
}

