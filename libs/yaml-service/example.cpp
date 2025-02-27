#include <iostream>
#include "yaml-service.hpp"

int main(int argc, char** argv) {
    try {
        YamlService yamlService(argv[1]);

        // 1. Retrieve basic values
        std::string host = yamlService.get_string_value({"server", "host"});
        int32_t port     = yamlService.get_int_value({"server", "port"});
        bool enableSSL   = yamlService.get_bool_value({"server", "enable_ssl"});
        std::cout << "Server host: " << host << "\n";
        std::cout << "Server port: " << port << "\n";
        std::cout << "SSL enabled: " << (enableSSL ? "true" : "false") << "\n\n";

        // 2. Nested structure: database credentials
        std::string dbUser = yamlService.get_string_value(
                {"database", "credentials", "user"});
        std::string dbPass = yamlService.get_string_value(
                {"database", "credentials", "password"});
        int32_t maxConns = yamlService.get_int_value({"database", "max_connections"});

        std::cout << "Database User: " << dbUser << "\n";
        std::cout << "Database Pass: " << dbPass << "\n";
        std::cout << "Max Connections: " << maxConns << "\n\n";

        // 3. Reading arrays
        std::vector<int> intList = yamlService.get_int_array({"numbers", "list_of_ints"});
        std::vector<float> floatList = yamlService.get_float_array({"numbers", "list_of_floats"});

        std::cout << "List of ints: ";
        for (int val : intList) {
            std::cout << val << " ";
        }
        std::cout << "\n";

        std::cout << "List of floats: ";
        for (float f : floatList) {
            std::cout << f << " ";
        }
        std::cout << "\n\n";

        // 4. Reading a top-level string
        std::string message = yamlService.get_string_value({"message"});
        std::cout << "Message: " << message << "\n\n";

        // 5. Using tryGetValue (returns std::nullopt if missing)
        auto optionalValue = yamlService.try_get_value<int>({"nonexistent", "path"});
        if (!optionalValue.has_value()) {
            std::cout << "Optional value was missing or of wrong type.\n";
        }

    } catch (const std::exception& ex) {
        std::cerr << "Error while reading YAML: " << ex.what() << std::endl;
    }

    return 0;
}

