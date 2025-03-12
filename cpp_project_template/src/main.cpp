#include "nodes/motor.hpp"
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>

using namespace nodes;


// Funkcia na čítanie klávesnice
int get_key() {
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);    // vypneme režim kanonického vstupu a echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto motor_controller = std::make_shared<MotorController>();
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(motor_controller);

    std::thread executor_thread([executor]() { executor->spin(); });

    std::vector<uint8_t> motor_speeds = {0, 0}; // Počiatočné rýchlosti motorov

    while (rclcpp::ok()) {
        int key = get_key();
        if (key == 27) {  // ESC key to exit
            break;
        } else if (key == 38) {  // Šípka hore
            motor_speeds[0] += 10;
            motor_speeds[1] += 10;
            motor_controller->set_motor_speeds({static_cast<float>(motor_speeds[0]), static_cast<float>(motor_speeds[1])});
        } else if (key == 40) {  // Šípka dole
            motor_speeds[0] -= 10;
            motor_speeds[1] -= 10;
            motor_controller->set_motor_speeds({static_cast<float>(motor_speeds[0]), static_cast<float>(motor_speeds[1])});
        } else if (key == 39) {  // Šípka vpravo
            motor_speeds[0] += 5;
            motor_controller->set_motor_speeds({static_cast<float>(motor_speeds[0]), static_cast<float>(motor_speeds[1])});
        } else if (key == 37) {  // Šípka vlavo
            motor_speeds[1] += 5;
            motor_controller->set_motor_speeds({static_cast<float>(motor_speeds[0]), static_cast<float>(motor_speeds[1])});
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}



