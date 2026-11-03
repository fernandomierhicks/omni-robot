// A flat JSON object writer for the link's 1 Hz status strings (/mcu/link, /mcu/clock): keys in insertion order,
// null for a missing value, numbers rounded as the Python nodes round them.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

namespace omni {

class JsonObject {
public:
    JsonObject& null(const std::string& k) { return raw(k, "null"); }
    JsonObject& boolean(const std::string& k, bool v) { return raw(k, v ? "true" : "false"); }
    JsonObject& integer(const std::string& k, int64_t v) { return raw(k, std::to_string(v)); }
    JsonObject& number(const std::string& k, double v, int decimals) {
        if (!std::isfinite(v)) return null(k);
        char b[64];
        std::snprintf(b, sizeof b, "%.*f", decimals, v);
        return raw(k, b);
    }
    JsonObject& number(const std::string& k, std::optional<double> v, int decimals) {
        return v ? number(k, *v, decimals) : null(k);
    }
    JsonObject& string(const std::string& k, const std::string& v) { return raw(k, quote(v)); }
    JsonObject& strings(const std::string& k, const std::vector<std::string>& v) {
        std::string s = "[";
        for (size_t i = 0; i < v.size(); ++i) s += (i ? ", " : "") + quote(v[i]);
        return raw(k, s + "]");
    }
    JsonObject& raw(const std::string& k, const std::string& v) {
        body_ += (body_.empty() ? "" : ", ") + quote(k) + ": " + v;
        return *this;
    }
    std::string str() const { return "{" + body_ + "}"; }

    static std::string quote(const std::string& s) {
        std::string o = "\"";
        for (char c : s) {
            if (c == '"' || c == '\\') o += '\\', o += c;
            else if ((unsigned char)c < 0x20) { char b[8]; std::snprintf(b, sizeof b, "\\u%04x", c); o += b; }
            else o += c;
        }
        return o + "\"";
    }

private:
    std::string body_;
};

}  // namespace omni
