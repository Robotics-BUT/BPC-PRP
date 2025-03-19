// line_sensor_listener.cpp
#include "nodes/line.hpp"

using namespace nodes;

LineSensorListener::LineSensorListener() : Node("line_sensor_listener"), Kp(0.014), Kd(0.004), base_speed(132), last_error(0.0)
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

        float right_motor_speed;
        float left_motor_speed;

        if (error > 0 ) {
             right_motor_speed = base_speed;
             left_motor_speed = base_speed - (Kp * error + Kd * d_error);
        }
        else if (error < 0) {
             right_motor_speed = base_speed + (Kp * error + Kd * d_error);
             left_motor_speed = base_speed;
        }
        else {
             right_motor_speed = base_speed;
             left_motor_speed = base_speed ;
        }

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

