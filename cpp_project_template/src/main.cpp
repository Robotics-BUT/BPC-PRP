#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.h"
#include "nodes/io_node.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <thread>
#include <chrono>
#include <cmath>
#include <termios.h>
#include <unistd.h>

class LedController : public rclcpp::Node {
public:
    LedController() : Node("led_controller") {
        led_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/rgb_leds", 10);
        motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/set_motor_speeds", 10);
    }

    void set_leds(const std::vector<uint8_t>& colors) {
        std_msgs::msg::UInt8MultiArray msg;
        msg.data = colors;
        led_publisher_->publish(msg);
    }

    void set_motor_speeds(const std::vector<uint8_t>& speeds) {
        std_msgs::msg::UInt8MultiArray msg;
        msg.data = speeds;
        motor_publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr led_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_publisher_;
};

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
    auto node = std::make_shared<nodes::IoNode>();
    auto led_controller = std::make_shared<LedController>();
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->add_node(led_controller);

    std::thread executor_thread([executor]() { executor->spin(); });

    std::vector<uint8_t> motor_speeds = {0, 0}; // Počiatočné rýchlosti motorov

    while (rclcpp::ok()) {
        int button = node->get_button_pressed();
        if (button != -1) {
            switch (button) {
                case 1:
                    RCLCPP_INFO(node->get_logger(), "Button 1 pressed - Turning all LEDs on");
                    led_controller->set_leds({255, 255, 255});
                    break;
                case 2:
                    RCLCPP_INFO(node->get_logger(), "Button 2 pressed - Cycling LED colors");
                    for (int i = 0; i < 10; ++i) {
                        led_controller->set_leds({255, 0, 0});
                        std::this_thread::sleep_for(std::chrono::milliseconds(300));
                        led_controller->set_leds({0, 255, 0});
                        std::this_thread::sleep_for(std::chrono::milliseconds(300));
                        led_controller->set_leds({0, 0, 255});
                        std::this_thread::sleep_for(std::chrono::milliseconds(300));
                    }
                    break;
                case 3:
                    RCLCPP_INFO(node->get_logger(), "Button 3 pressed - Modulating LED intensity");
                    for (int i = 0; i < 100; ++i) {
                        uint8_t r = static_cast<uint8_t>(127.5 * (1 + sin(2 * M_PI * i / 100)));
                        uint8_t g = static_cast<uint8_t>(127.5 * (1 + sin(2 * M_PI * (i / 100.0 - 1.0 / 3))));
                        uint8_t b = static_cast<uint8_t>(127.5 * (1 + sin(2 * M_PI * (i / 100.0 - 2.0 / 3))));
                        led_controller->set_leds({r, g, b});
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                    break;
                default:
                    RCLCPP_INFO(node->get_logger(), "Unknown button pressed: %d", button);
                    break;
            }
        }

        // Čítanie klávesov pre šípky
        int key = get_key();
        if (key == 27) {  // ESC key to exit
            break;
        } else if (key == 65) {  // Šípka hore
            motor_speeds[0] += 10;  // Zvyšujeme rýchlosť motora 1
            motor_speeds[1] += 10;  // Zvyšujeme rýchlosť motora 2
            RCLCPP_INFO(node->get_logger(), "Speed up motors: %d %d", motor_speeds[0], motor_speeds[1]);
            led_controller->set_motor_speeds(motor_speeds);
        } else if (key == 66) {  // Šípka dole
            motor_speeds[0] -= 10;  // Znižujeme rýchlosť motora 1
            motor_speeds[1] -= 10;  // Znižujeme rýchlosť motora 2
            RCLCPP_INFO(node->get_logger(), "Speed down motors: %d %d", motor_speeds[0], motor_speeds[1]);
            led_controller->set_motor_speeds(motor_speeds);
        } else if (key == 67) {  // Šípka vpravo
            motor_speeds[0] += 5;  // Trochu zvýšime motor 1
            RCLCPP_INFO(node->get_logger(), "Increase motor 1: %d", motor_speeds[0]);
            led_controller->set_motor_speeds(motor_speeds);
        } else if (key == 68) {  // Šípka vlavo
            motor_speeds[1] += 5;  // Trochu zvýšime motor 2
            RCLCPP_INFO(node->get_logger(), "Increase motor 2: %d", motor_speeds[1]);
            led_controller->set_motor_speeds(motor_speeds);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}



