#pragma once

#include <string>
#include <optional>
#include <initializer_list>
#include <stdexcept>
#include <string_view>
#include <toml++/toml.h>

class TomlLoader {
public:
    explicit TomlLoader(std::string path)
        : toml_path_(std::move(path)), root_(toml::parse_file(toml_path_)) {}

    void load(const std::string& path) { toml_path_ = path; root_ = toml::parse_file(toml_path_); }
    void reload() { root_ = toml::parse_file(toml_path_); }

    // ---- Scalars ----
    template <typename T>
    T get(std::initializer_list<std::string_view> keys) const {
        const toml::node* n = node_at(keys);
        if (auto v = n->value<T>()) return *v;
        throw std::runtime_error("Type mismatch at TOML path");
    }

    template <typename T>
    std::optional<T> get_opt(std::initializer_list<std::string_view> keys) const {
        const toml::node* n = node_at_or_null(keys);
        if (!n) return std::nullopt;
        if (auto v = n->value<T>()) return *v;
        return std::nullopt;
    }

    template <typename T>
    T get_or(std::initializer_list<std::string_view> keys, T fallback) const {
        if (auto v = get_opt<T>(keys)) return *v;
        return fallback;
    }

    // ---- Arrays ----
    template <typename T>
    std::vector<T> get_vec(std::initializer_list<std::string_view> keys) const {
        const toml::node* n = node_at(keys);
        auto* arr = n->as_array();
        if (!arr) throw std::runtime_error("Expected array at TOML path");
        return array_to_vector<T>(*arr);
    }

    template <typename T>
    std::optional<std::vector<T>> get_vec_opt(std::initializer_list<std::string_view> keys) const {
        const toml::node* n = node_at_or_null(keys);
        if (!n) return std::nullopt;
        auto* arr = n->as_array();
        if (!arr) return std::nullopt;
        return array_to_vector<T>(*arr);
    }

    template <typename T>
    std::vector<T> get_vec_or(std::initializer_list<std::string_view> keys,
                              std::vector<T> fallback) const {
        if (auto v = get_vec_opt<T>(keys)) return *v;
        return fallback;
    }

private:
    template <typename T>
    static std::vector<T> array_to_vector(const toml::array& a) {
        std::vector<T> out;
        out.reserve(a.size());
        for (const toml::node& elem : a) {
            if (auto val = elem.value<T>()) out.push_back(*val);
            else throw std::runtime_error("Array element type mismatch");
        }
        return out;
    }

    const toml::node* node_at(std::initializer_list<std::string_view> keys) const {
        const toml::node* cur = &root_;
        for (auto k : keys) {
            auto* tbl = cur->as_table();
            if (!tbl) throw std::runtime_error("Expected a table in TOML path");
            cur = tbl->get(std::string{k});
            if (!cur) throw std::runtime_error("Missing key in TOML: " + std::string(k));
        }
        return cur;
    }

    const toml::node* node_at_or_null(std::initializer_list<std::string_view> keys) const {
        const toml::node* cur = &root_;
        for (auto k : keys) {
            auto* tbl = cur->as_table();
            if (!tbl) return nullptr;
            cur = tbl->get(std::string{k});
            if (!cur) return nullptr;
        }
        return cur;
    }

    std::string toml_path_;
    toml::table root_;
};