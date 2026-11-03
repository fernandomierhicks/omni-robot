// A JSON value for the catch code (design.md 4.9): reads config/robot_v11.json and /catch/setup, writes /catch/status's
// `last`, /catch/field and /catch/perception as Python's json.dumps does (keys in insertion order, ", " and ": ",
// floats as repr() prints them, NaN for a NaN), so the console reads the same text from either implementation.
#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace omni {

class Json {
public:
    enum Type { NUL, BOOL, INT, FLOAT, STR, ARR, OBJ };

    Json() = default;
    Json(std::nullptr_t) {}
    Json(bool b) : type_(BOOL), b_(b) {}
    Json(int v) : type_(INT), i_(v) {}
    Json(int64_t v) : type_(INT), i_(v) {}
    Json(double v) : type_(FLOAT), d_(v) {}
    Json(const char* s) : type_(STR), s_(s) {}
    Json(std::string s) : type_(STR), s_(std::move(s)) {}
    static Json array() { Json j; j.type_ = ARR; return j; }
    static Json object() { Json j; j.type_ = OBJ; return j; }
    static Json floats(const std::vector<double>& v);
    static Json parse(const std::string& text);       // throws std::runtime_error

    Type type() const { return type_; }
    bool is_null() const { return type_ == NUL; }
    bool is_number() const { return type_ == INT || type_ == FLOAT; }
    bool truthy() const;                               // Python's bool() of the value
    double num() const;                                // a number (int or float); throws otherwise
    int64_t integer() const { return type_ == INT ? i_ : (int64_t)num(); }
    bool boolean() const { return type_ == BOOL ? b_ : truthy(); }
    const std::string& str() const;
    std::vector<double> nums() const;                  // an array of numbers
    std::vector<std::vector<double>> rows() const;     // an array of arrays of numbers

    // arrays
    size_t size() const { return type_ == ARR ? arr_.size() : type_ == OBJ ? obj_.size() : 0; }
    const Json& operator[](size_t i) const { return arr_.at(i); }
    Json& push(Json v) { arr_.push_back(std::move(v)); return arr_.back(); }
    // objects: set keeps the first position of a key, as a Python dict does
    bool has(const std::string& k) const { return find(k) != nullptr; }
    const Json& operator[](const std::string& k) const;
    const Json& get(const std::string& k) const;      // null if missing (dict.get)
    Json& set(const std::string& k, Json v);
    const std::vector<std::pair<std::string, Json>>& items() const { return obj_; }

    std::string dump() const;

private:
    const Json* find(const std::string& k) const;
    Type type_ = NUL;
    bool b_ = false;
    int64_t i_ = 0;
    double d_ = 0;
    std::string s_;
    std::vector<Json> arr_;
    std::vector<std::pair<std::string, Json>> obj_;
};

// repr() of a Python float: the shortest digits that read back the same, fixed notation for 1e-4 <= |v| < 1e16
std::string py_float(double v);
// Python's round(v, nd) (to the nearest decimal, ties as the exact binary value falls)
double py_round(double v, int nd);

}  // namespace omni
