#include "nodes/motor.hpp"
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>

using namespace nodes;
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto motor_controller = std::make_shared<MotorController>();
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(motor_controller);

    std::thread executor_thread([executor]() { executor->spin(); });
    std::vector<uint8_t> motor_speeds = {0, 0}; // Počiatočné rýchlosti motorov

    while (rclcpp::ok()) {



        motor_speeds[0] = 140;
        motor_speeds[1] = 140;

        // Nastaví rýchlosť motorov
        motor_controller->set_motor_speeds({static_cast<float>(motor_speeds[0]), static_cast<float>(motor_speeds[1])});



    }

    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}
