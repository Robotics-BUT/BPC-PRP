#include "rclcpp/rclcpp.hpp"
#include "nodes/button.hpp"
#include "nodes/line.hpp"

using namespace nodes;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto button_listener = std::make_shared<ButtonListener>();
    auto line_sensor_listener = std::make_shared<LineSensorListener>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(button_listener);

    bool was_active = false;


    while (rclcpp::ok())
    {
        executor.spin_some();

        bool is_active = button_listener->is_active();

        if (is_active && !was_active)
        {
            RCLCPP_INFO(button_listener->get_logger(), " Spúšťam LineSensorListener");
            executor.add_node(line_sensor_listener);
        }
        else if (!is_active && was_active)
        {
            RCLCPP_INFO(button_listener->get_logger(), " Zastavujem LineSensorListener");
            executor.remove_node(line_sensor_listener);
            line_sensor_listener->reset_integral();
        }

        was_active = is_active;
    }

    rclcpp::shutdown();
    return 0;
}






