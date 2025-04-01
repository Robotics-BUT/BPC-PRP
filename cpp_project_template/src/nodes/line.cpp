// line_sensor_listener.cpp
#include "nodes/line.hpp"

using namespace nodes;

LineSensorListener::LineSensorListener() : Node("line_sensor_listener"), Kp(0.07), Kd(0.0015),Ki(0.001), base_speed(132), last_error(0.0)
{
    line_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
        "/bpc_prp_robot/line_sensors", 10,
        std::bind(&LineSensorListener::on_line_sensors_msg, this, std::placeholders::_1));

    motor_controller_ = std::make_shared<MotorController>();
}

void LineSensorListener::reset_integral() {
    integral_=0.0f;
}

void LineSensorListener::on_line_sensors_msg(std::shared_ptr<std_msgs::msg::UInt16MultiArray> msg)
{
    if (msg->data.size() >= 2)
    {
        float left_value = static_cast<float>(msg->data[0]);
        float right_value = static_cast<float>(msg->data[1]);

        float error = left_value - right_value;
        float error_avg=error+last_error/2;
        float d_error = error_avg - last_error;

        // +limitace na integralu
        integral_+=error_avg;
        if (integral_>2000){integral_=2000;}
        if (integral_<-2000){integral_=-2000;}
        RCLCPP_INFO(this->get_logger(), "Left Sensor: %f, Right Sensor: %f Integral: %f", left_value, right_value, integral_);

        //pocitani akcniho zasahu
        float speed_diff=(Kp * error_avg + Kd * d_error + Ki*integral_);
        //float speed_diff=(Kp * error);
        // limitace akcniho zasahu
        if (speed_diff>base_speed-128){speed_diff=base_speed-128;}  // moc velky kladny
        else if (speed_diff<128-base_speed){speed_diff=128-base_speed;} // moc velky zaporny

        float right_motor_speed;
        float left_motor_speed;
        float sensor_threshhold=300;    // jsem na krizi -> nechci zatacet

        if (error==0 || (left_value>sensor_threshhold && right_value>sensor_threshhold)) {
            // dostal jsem se na kriz nebo jedu dobre, tak chci rovne
            right_motor_speed = base_speed;//+Ki*integral_;
            left_motor_speed = base_speed;//-Ki*integral_;
        }
        else if (error > 0 ) {
             right_motor_speed = base_speed+ 0.3*speed_diff;//
             left_motor_speed = base_speed - speed_diff;
        }
        else if (error < 0) {
             right_motor_speed = base_speed + speed_diff;
             left_motor_speed = base_speed- 0.3*speed_diff;//
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

