#include "rclcpp/rclcpp.hpp"
#include "nodes/motor.hpp"  // Predpokladáme, že máš motorový uzol
#include "nodes/line.hpp"

using namespace nodes;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto line_sensor_listener = std::make_shared<LineSensorListener>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(line_sensor_listener);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}




