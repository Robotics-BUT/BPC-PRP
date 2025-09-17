#include "toml_loader.hpp"
#include <iostream>

int main() {
    try {
        TomlLoader tl{"../../../libs/toml_loader/config.toml"};

        int port = tl.get<int>({"server", "port"});
        std::string user = tl.get_or<std::string>({"auth", "user"}, "guest");
        bool debug = tl.get_or<bool>({"server", "debug"}, false);

        std::cout << "Server port: " << port << "\n";
        std::cout << "Auth user:   " << user << "\n";
        std::cout << "Debug mode:  " << std::boolalpha << debug << "\n";

        if (auto gains = tl.get_vec_opt<double>({"controller", "gains"})) {
            std::cout << "Controller gains: ";
            for (double g : *gains) std::cout << g << " ";
            std::cout << "\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}