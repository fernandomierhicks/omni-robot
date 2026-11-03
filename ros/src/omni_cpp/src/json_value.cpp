#include "omni_cpp/json_value.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace omni {

namespace {

struct Parser {
    const std::string& s;
    size_t i = 0;

    [[noreturn]] void fail(const char* what) const {
        throw std::runtime_error(std::string("json: ") + what + " at " + std::to_string(i));
    }
    void ws() {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t')) ++i;
    }
    bool lit(const char* w) {
        size_t n = std::strlen(w);
        if (s.compare(i, n, w) == 0) {
            i += n;
            return true;
        }
        return false;
    }
    static void utf8(std::string& o, unsigned cp) {
        if (cp < 0x80) {
            o += (char)cp;
        } else if (cp < 0x800) {
            o += (char)(0xC0 | (cp >> 6)), o += (char)(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            o += (char)(0xE0 | (cp >> 12)), o += (char)(0x80 | ((cp >> 6) & 0x3F)), o += (char)(0x80 | (cp & 0x3F));
        } else {
            o += (char)(0xF0 | (cp >> 18)), o += (char)(0x80 | ((cp >> 12) & 0x3F));
            o += (char)(0x80 | ((cp >> 6) & 0x3F)), o += (char)(0x80 | (cp & 0x3F));
        }
    }
    unsigned hex4() {
        if (i + 4 > s.size()) fail("bad \\u");
        unsigned v = (unsigned)std::strtoul(s.substr(i, 4).c_str(), nullptr, 16);
        i += 4;
        return v;
    }
    std::string string() {
        if (s[i] != '"') fail("expected a string");
        ++i;
        std::string o;
        while (i < s.size() && s[i] != '"') {
            char c = s[i++];
            if (c != '\\') {
                o += c;
                continue;
            }
            if (i >= s.size()) fail("bad escape");
            char e = s[i++];
            switch (e) {
                case 'n': o += '\n'; break;
                case 't': o += '\t'; break;
                case 'r': o += '\r'; break;
                case 'b': o += '\b'; break;
                case 'f': o += '\f'; break;
                case 'u': {
                    unsigned cp = hex4();
                    if (cp >= 0xD800 && cp < 0xDC00 && s.compare(i, 2, "\\u") == 0) {
                        i += 2;
                        unsigned lo = hex4();
                        cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                    }
                    utf8(o, cp);
                    break;
                }
                default: o += e;
            }
        }
        if (i >= s.size()) fail("unterminated string");
        ++i;
        return o;
    }
    Json value() {
        ws();
        if (i >= s.size()) fail("unexpected end");
        char c = s[i];
        if (c == '{') {
            ++i;
            Json o = Json::object();
            ws();
            if (s[i] == '}') return ++i, o;
            while (true) {
                ws();
                std::string k = string();
                ws();
                if (s[i++] != ':') fail("expected ':'");
                o.set(k, value());
                ws();
                if (s[i] == ',') { ++i; continue; }
                if (s[i] == '}') { ++i; return o; }
                fail("expected ',' or '}'");
            }
        }
        if (c == '[') {
            ++i;
            Json a = Json::array();
            ws();
            if (s[i] == ']') return ++i, a;
            while (true) {
                a.push(value());
                ws();
                if (s[i] == ',') { ++i; continue; }
                if (s[i] == ']') { ++i; return a; }
                fail("expected ',' or ']'");
            }
        }
        if (c == '"') return Json(string());
        if (lit("true")) return Json(true);
        if (lit("false")) return Json(false);
        if (lit("null")) return Json();
        if (lit("NaN")) return Json(std::nan(""));
        if (lit("Infinity")) return Json(HUGE_VAL);
        if (lit("-Infinity")) return Json(-HUGE_VAL);
        size_t j = i;
        bool flt = false;
        if (s[j] == '-') ++j;
        while (j < s.size() && (std::isdigit((unsigned char)s[j]) || s[j] == '.' || s[j] == 'e' || s[j] == 'E' ||
                                s[j] == '+' || s[j] == '-')) {
            flt = flt || s[j] == '.' || s[j] == 'e' || s[j] == 'E';
            ++j;
        }
        if (j == i) fail("unexpected character");
        std::string t = s.substr(i, j - i);
        i = j;
        if (flt) return Json(std::strtod(t.c_str(), nullptr));
        return Json((int64_t)std::strtoll(t.c_str(), nullptr, 10));
    }
};

void escape(std::string& o, const std::string& s) {
    o += '"';
    for (size_t k = 0; k < s.size(); ++k) {
        unsigned char c = (unsigned char)s[k];
        char b[16];
        if (c == '"') o += "\\\"";
        else if (c == '\\') o += "\\\\";
        else if (c == '\n') o += "\\n";
        else if (c == '\r') o += "\\r";
        else if (c == '\t') o += "\\t";
        else if (c == '\b') o += "\\b";
        else if (c == '\f') o += "\\f";
        else if (c < 0x20) std::snprintf(b, sizeof b, "\\u%04x", c), o += b;
        else if (c < 0x80) o += (char)c;
        else {                                     // json.dumps' ensure_ascii: UTF-8 -> \uXXXX (surrogate pairs)
            unsigned cp = 0;
            int extra = c >= 0xF0 ? 3 : c >= 0xE0 ? 2 : 1;
            cp = c & (0x3F >> extra);
            for (int e = 0; e < extra && k + 1 < s.size(); ++e) cp = (cp << 6) | ((unsigned char)s[++k] & 0x3F);
            if (cp >= 0x10000) {
                cp -= 0x10000;
                std::snprintf(b, sizeof b, "\\u%04x\\u%04x", 0xD800 + (cp >> 10), 0xDC00 + (cp & 0x3FF));
            } else {
                std::snprintf(b, sizeof b, "\\u%04x", cp);
            }
            o += b;
        }
    }
    o += '"';
}

void dump_to(std::string& o, const Json& j) {
    switch (j.type()) {
        case Json::NUL: o += "null"; break;
        case Json::BOOL: o += j.boolean() ? "true" : "false"; break;
        case Json::INT: o += std::to_string(j.integer()); break;
        case Json::FLOAT: o += py_float(j.num()); break;
        case Json::STR: escape(o, j.str()); break;
        case Json::ARR:
            o += '[';
            for (size_t k = 0; k < j.size(); ++k) {
                if (k) o += ", ";
                dump_to(o, j[k]);
            }
            o += ']';
            break;
        case Json::OBJ: {
            o += '{';
            bool first = true;
            for (auto& kv : j.items()) {
                if (!first) o += ", ";
                first = false;
                escape(o, kv.first);
                o += ": ";
                dump_to(o, kv.second);
            }
            o += '}';
            break;
        }
    }
}

const Json kNull;

}  // namespace

Json Json::parse(const std::string& text) {
    Parser p{text};
    Json v = p.value();
    p.ws();
    if (p.i != text.size()) p.fail("trailing characters");
    return v;
}

Json Json::floats(const std::vector<double>& v) {
    Json a = array();
    for (double x : v) a.push(Json(x));
    return a;
}

bool Json::truthy() const {
    switch (type_) {
        case NUL: return false;
        case BOOL: return b_;
        case INT: return i_ != 0;
        case FLOAT: return d_ != 0.0;
        case STR: return !s_.empty();
        case ARR: return !arr_.empty();
        case OBJ: return !obj_.empty();
    }
    return false;
}

double Json::num() const {
    if (type_ == FLOAT) return d_;
    if (type_ == INT) return (double)i_;
    if (type_ == BOOL) return b_ ? 1.0 : 0.0;
    throw std::runtime_error("json: not a number");
}

const std::string& Json::str() const {
    if (type_ != STR) throw std::runtime_error("json: not a string");
    return s_;
}

std::vector<double> Json::nums() const {
    std::vector<double> v;
    for (auto& x : arr_) v.push_back(x.num());
    return v;
}

std::vector<std::vector<double>> Json::rows() const {
    std::vector<std::vector<double>> v;
    for (auto& x : arr_) v.push_back(x.nums());
    return v;
}

const Json* Json::find(const std::string& k) const {
    for (auto& kv : obj_)
        if (kv.first == k) return &kv.second;
    return nullptr;
}

const Json& Json::operator[](const std::string& k) const {
    const Json* v = find(k);
    if (!v) throw std::runtime_error("json: no key '" + k + "'");
    return *v;
}

const Json& Json::get(const std::string& k) const {
    const Json* v = find(k);
    return v ? *v : kNull;
}

Json& Json::set(const std::string& k, Json v) {
    for (auto& kv : obj_)
        if (kv.first == k) return kv.second = std::move(v);
    obj_.emplace_back(k, std::move(v));
    return obj_.back().second;
}

std::string Json::dump() const {
    std::string o;
    dump_to(o, *this);
    return o;
}

std::string py_float(double v) {
    if (std::isnan(v)) return "NaN";
    if (std::isinf(v)) return v > 0 ? "Infinity" : "-Infinity";
    char b[40];
    int p = 1;
    for (; p <= 17; ++p) {
        std::snprintf(b, sizeof b, "%.*e", p - 1, v);
        if (std::strtod(b, nullptr) == v) break;
    }
    // b = [-]d[.ddd]e(+|-)XX
    std::string s(b), sign, digits;
    size_t k = 0;
    if (s[0] == '-') sign = "-", k = 1;
    size_t e = s.find('e');
    for (size_t q = k; q < e; ++q)
        if (s[q] != '.') digits += s[q];
    int ex = std::atoi(s.c_str() + e + 1);
    while (digits.size() > 1 && digits.back() == '0') digits.pop_back();
    std::string o = sign;
    if (ex >= -4 && ex < 16) {
        if (ex >= 0) {
            std::string ip = digits.substr(0, std::min(digits.size(), (size_t)ex + 1));
            while ((int)ip.size() < ex + 1) ip += '0';
            std::string fp = digits.size() > (size_t)ex + 1 ? digits.substr(ex + 1) : "0";
            o += ip + "." + fp;
        } else {
            o += "0." + std::string(-ex - 1, '0') + digits;
        }
    } else {
        o += digits.substr(0, 1);
        if (digits.size() > 1) o += "." + digits.substr(1);
        char x[16];
        std::snprintf(x, sizeof x, "e%c%02d", ex < 0 ? '-' : '+', std::abs(ex));
        o += x;
    }
    return o;
}

double py_round(double v, int nd) {
    if (!std::isfinite(v)) return v;
    if (nd >= 0 && nd <= 15) {
        // exactly, as CPython does it: |v| = M 2^e; round M 10^nd / 2^-e to an integer q, ties to even; q / 10^nd.
        // (printf's rounding of an exact tie differs between C libraries.)
        int e;
        double m = std::frexp(std::fabs(v), &e);
        uint64_t M = (uint64_t)std::ldexp(m, 53);
        e -= 53;
        if (e >= 0) return v;                              // an integer already
        uint64_t p10 = 1;
        for (int k = 0; k < nd; ++k) p10 *= 10;
        unsigned __int128 N = (unsigned __int128)M * p10, q = 0;
        int k = -e;
        if (k < 120) {
            q = N >> k;
            unsigned __int128 rem = N - (q << k), half = (unsigned __int128)1 << (k - 1);
            if (rem > half || (rem == half && (q & 1))) ++q;
        }
        if (q < ((unsigned __int128)1 << 53)) return std::copysign((double)(uint64_t)q / (double)p10, v);
    }
    char b[400];
    std::snprintf(b, sizeof b, "%.*f", nd, v);
    return std::strtod(b, nullptr);
}

}  // namespace omni
