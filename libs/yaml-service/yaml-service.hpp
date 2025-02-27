#pragma once

#include <string>
#include <vector>
#include <optional>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

class YamlService {
public:

    explicit YamlService(const std::string& yamlPath)
            : yaml_path_(std::move(yamlPath)),
              root_node_(YAML::LoadFile(yaml_path_))
    {}

    void load(const std::string& path) {
        root_node_ = YAML::LoadFile(path);
    }

    void reload() {
        root_node_ = YAML::LoadFile(yaml_path_);
    }

    template <typename T>
    T getValue(const std::vector<std::string>& keys) const {
        YAML::Node node = get_node(keys);
        return node.as<T>();
    }

    template <typename T>
    std::optional<T> try_get_value(const std::vector<std::string>& keys) const {
        try {
            YAML::Node node = get_node(keys);
            if (!node) {
                return std::nullopt;
            }
            return node.as<T>();
        } catch (const std::runtime_error&) {
            std::stringstream ss;
            ss << "Unable to read value with keys: ";
            for (const auto& key : keys) {
                ss << key << ".";
            }
            std::cerr << ss.str() << std::endl;
            return std::nullopt;
        }
    }

    uint32_t get_uint_value(const std::vector<std::string>& keys) const {
        return getValue<uint32_t>(keys);
    }

    int32_t get_int_value(const std::vector<std::string>& keys) const {
        return getValue<int32_t>(keys);
    }

    float get_float_value(const std::vector<std::string>& keys) const {
        return getValue<float>(keys);
    }

    double get_double_value(const std::vector<std::string>& keys) const {
        return getValue<double>(keys);
    }

    bool get_bool_value(const std::vector<std::string>& keys) const {
        return getValue<bool>(keys);
    }

    std::string get_string_value(const std::vector<std::string>& keys) const {
        return getValue<std::string>(keys);
    }

    std::vector<float> get_float_array(const std::vector<std::string>& keys) const {
        return getValue<std::vector<float>>(keys);
    }

    std::vector<int> get_int_array(const std::vector<std::string>& keys) const {
        return getValue<std::vector<int>>(keys);
    }

    std::vector<YAML::Node> get_node_array(const std::vector<std::string>& keys) const {
        return getValue<std::vector<YAML::Node>>(keys);
    }

private:
    YAML::Node get_node(const std::vector<std::string>& keys) const {
        YAML::Node current = Clone(root_node_);
        for (const auto& key : keys) {
            if (!current[key]) {
                throw std::runtime_error("Unable to read \"" + key + "\" from config file!");
            }
            current = current[key];
        }
        return current;
    }

private:
    std::string yaml_path_;
    YAML::Node root_node_;
};