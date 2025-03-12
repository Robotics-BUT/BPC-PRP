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


    // Vypočíta rozdiel medzi aktuálnymi a počiatočnými hodnotami enkodérov
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::vector<uint8_t> motor_speeds = {0, 0};


        motor_speeds[0] = 160;
        motor_speeds[1] = 160;

        // Nastaví rýchlosť motorov
        motor_controller->set_motor_speeds({static_cast<float>(motor_speeds[0]), static_cast<float>(motor_speeds[1])});




    rclcpp::shutdown();
    executor_thread.join();
    return 0;
    }



