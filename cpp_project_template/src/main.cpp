#include "rclcpp/rclcpp.hpp"
#include "nodes/button.hpp"
#include "nodes/lidar.hpp"
#include "nodes/imu_node.hpp"  // přidáno

using namespace nodes;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto button_listener = std::make_shared<ButtonListener>();
    auto lidar_filter_node = std::make_shared<LidarFilterNode>();
    auto imu_node = std::make_shared<ImuNode>();  // přidáno

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(button_listener);
    executor.add_node(imu_node);  // přidáno

    // Spustit kalibraci
    imu_node->setMode(ImuNodeMode::CALIBRATE);
    RCLCPP_INFO(imu_node->get_logger(), "Kalibrace IMU zahájena...");

    auto start_time = imu_node->now();

    bool was_active = false;

    while (rclcpp::ok())
    {
        executor.spin_some();

        auto now = imu_node->now();
        auto elapsed = (now - start_time).seconds();

        if (elapsed >= 2.0 && imu_node->getMode() == ImuNodeMode::CALIBRATE)
        {
            imu_node->setMode(ImuNodeMode::INTEGRATE);
            RCLCPP_INFO(imu_node->get_logger(), "Kalibrace dokončena. Přepínám do INTEGRATE módu.");
        }

        if (imu_node->getMode() == ImuNodeMode::INTEGRATE && static_cast<int>(elapsed) % 1 == 0)
        {
            auto yaw = imu_node->getIntegratedResults();

            RCLCPP_INFO_THROTTLE(
                imu_node->get_logger(),
                *imu_node->get_clock(),  // správně: clock, ne čas
                200,  // throttle duration v milisekundách
                "Aktuální úhel (yaw): %.4f rad", yaw
            );
        }

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
