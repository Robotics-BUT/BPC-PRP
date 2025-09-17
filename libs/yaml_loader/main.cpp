#include "yaml_loader.hpp"
#include <iostream>
#include <vector>

int main() {
    try {
        YamlLoader yl{"../../../libs/yaml_loader/config.yaml"};

        int port = yl.get<int>({"server", "port"});
        std::string user = yl.get_or<std::string>({"auth", "user"}, "guest");
        bool debug = yl.get_or<bool>({"server", "debug"}, false);

        std::cout << "Server port: " << port << "\n";
        std::cout << "Auth user:   " << user << "\n";
        std::cout << "Debug mode:  " << std::boolalpha << debug << "\n";

        if (auto gains = yl.get_vec_opt<double>({"controller", "gains"})) {
            std::cout << "Controller gains: ";
            for (double g : *gains) std::cout << g << " ";
            std::cout << "\n";
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}