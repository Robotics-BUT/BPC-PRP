#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "nodes/motor.hpp"  // Predpokladáme, že máš motorový uzol

using namespace nodes;

class LineSensorListener : public rclcpp::Node
{
public:
    LineSensorListener() : Node("line_sensor_listener")
    {
        // Predplatíme sa na tému /bpc_prp_robot/line_sensors
        line_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/bpc_prp_robot/line_sensors", 10,
            std::bind(&LineSensorListener::on_line_sensors_msg, this, std::placeholders::_1));

        // Vytvoríme motor controller uzol
        motor_controller_ = std::make_shared<MotorController>();
    }

private:
    void on_line_sensors_msg(std::shared_ptr<std_msgs::msg::UInt16MultiArray> msg)
    {
        if (msg->data.size() >= 2)
        {
            float left_value = static_cast<float>(msg->data[0]);
            float right_value = static_cast<float>(msg->data[1]);

            // Vypíš hodnoty zo senzorov do konzoly
            RCLCPP_INFO(this->get_logger(), "Left Sensor: %f, Right Sensor: %f", left_value, right_value);

            // Počiatočné rýchlosti motorov
            uint8_t left_motor_speed = 135;
            uint8_t right_motor_speed = 135;

            // Podmienky pre ovládanie motorov na základe senzorov
            if (left_value > 50 && right_value > 50)
            {
                // Ak oba senzory zachytávajú čiaru, nastavíme motory na hodnotu 140
                left_motor_speed = 135;
                right_motor_speed = 135;
            }
            else if (left_value > 50)
            {
                // Ak ľavý senzor zachytí čiaru, pravý motor sa zvýši o 5, ľavý sa zníži o 5
                left_motor_speed = 135 - 3;  // Zníženie ľavého motora o 5
                right_motor_speed = 135 + 3; // Zvýšenie pravého motora o 5
            }
            else if (right_value > 50)
            {
                // Ak pravý senzor zachytí čiaru, ľavý motor sa zvýši o 5, pravý sa zníži o 5
                left_motor_speed = 135 + 3;  // Zníženie ľavého motora o 5
                right_motor_speed = 135 - 3;// Zníženie pravého motora o 5
            }

            // Nastavíme rýchlosti motorov
            motor_controller_->set_motor_speeds({static_cast<float>(left_motor_speed), static_cast<float>(right_motor_speed)});
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Invalid sensor data received.");
        }
    }

    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;
    std::shared_ptr<MotorController> motor_controller_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Vytvorenie uzla, ktorý bude predplatený na tému
    auto line_sensor_listener = std::make_shared<LineSensorListener>();

    // Vytvorenie exekútora a priradenie uzla
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(line_sensor_listener);

    // Spustenie exekútora
    executor.spin();

    rclcpp::shutdown();
    return 0;
}





