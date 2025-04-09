#include "rclcpp/rclcpp.hpp"
#include "nodes/button.hpp"
#include "nodes/lidar.hpp"

using namespace nodes;

using namespace nodes;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto button_listener = std::make_shared<ButtonListener>();
    auto lidar_filter_node = std::make_shared<LidarFilterNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(button_listener);

    bool was_active = false;

    while (rclcpp::ok())
    {
        executor.spin_some();

        bool is_active = button_listener->is_active();

        if (is_active && !was_active)
        {
            RCLCPP_INFO(button_listener->get_logger(), "Spúšťam LidarFilterNode");
            executor.add_node(lidar_filter_node);
        }
        else if (!is_active && was_active)
        {
            RCLCPP_INFO(button_listener->get_logger(), "Zastavujem LidarFilterNode");
            executor.remove_node(lidar_filter_node);
        }

        was_active = is_active;
    }

    rclcpp::shutdown();
    return 0;
}






