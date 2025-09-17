#pragma once
#include <string>
#include <optional>
#include <initializer_list>
#include <string_view>
#include <vector>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

class YamlLoader {
public:
    explicit YamlLoader(std::string path)
        : yaml_path_(std::move(path)), root_(YAML::LoadFile(yaml_path_)) {}

    void load(const std::string& path) { yaml_path_ = path; root_ = YAML::LoadFile(yaml_path_); }
    void reload() { root_ = YAML::LoadFile(yaml_path_); }

    // ---- Scalars ----
    template <typename T>
    T get(std::initializer_list<std::string_view> keys) const {
        const YAML::Node n = node_at(keys);
        if (!n) throw std::runtime_error("Missing value at YAML path");
        try { return n.as<T>(); }
        catch (...) { throw std::runtime_error("Type mismatch at YAML path"); }
    }

    template <typename T>
    std::optional<T> get_opt(std::initializer_list<std::string_view> keys) const {
        const YAML::Node n = node_at_or_null(keys);
        if (!n) return std::nullopt;
        try { return n.as<T>(); }
        catch (...) { return std::nullopt; }
    }

    template <typename T>
    T get_or(std::initializer_list<std::string_view> keys, T fallback) const {
        if (auto v = get_opt<T>(keys)) return *v;
        return fallback;
    }

    // ---- Arrays (sequences of scalars) ----
    template <typename T>
    std::vector<T> get_vec(std::initializer_list<std::string_view> keys) const {
        const YAML::Node n = node_at(keys);
        if (!n || !n.IsSequence())
            throw std::runtime_error("Expected a sequence at YAML path");
        return sequence_to_vector<T>(n);
    }

    template <typename T>
    std::optional<std::vector<T>> get_vec_opt(std::initializer_list<std::string_view> keys) const {
        const YAML::Node n = node_at_or_null(keys);
        if (!n || !n.IsSequence()) return std::nullopt;
        try { return sequence_to_vector<T>(n); }
        catch (...) { return std::nullopt; }
    }

    template <typename T>
    std::vector<T> get_vec_or(std::initializer_list<std::string_view> keys,
                              std::vector<T> fallback) const {
        if (auto v = get_vec_opt<T>(keys)) return *v;
        return fallback;
    }

private:
    template <typename T>
    static std::vector<T> sequence_to_vector(const YAML::Node& seq) {
        std::vector<T> out;
        out.reserve(seq.size());
        for (const auto& item : seq) {
            out.push_back(item.as<T>()); // throws on mismatch
        }
        return out;
    }

    YAML::Node node_at(std::initializer_list<std::string_view> keys) const {
        YAML::Node cur = root_;
        for (auto k : keys) {
            if (!cur.IsMap())
                throw std::runtime_error("Expected a map while traversing YAML path");
            cur = cur[std::string{k}];
            if (!cur)
                throw std::runtime_error("Missing key in YAML: " + std::string(k));
        }
        return cur;
    }

    YAML::Node node_at_or_null(std::initializer_list<std::string_view> keys) const {
        YAML::Node cur = root_;
        for (auto k : keys) {
            if (!cur || !cur.IsMap()) return YAML::Node{};
            cur = cur[std::string{k}];
            if (!cur) return YAML::Node{};
        }
        return cur;
    }

    std::string yaml_path_;
    YAML::Node root_;
};