// A 200-line JSON reader for the SIL harness's three fixed schemas.
//
// Host-only (never compiled for the ESP32) and deliberately dependency-free:
// the firmware's acceptance gate is "a laptop with a C++ compiler", and the
// design doc asks for the weight sidecar to be read with "no new
// dependencies". It parses the JSON subset those files use -- objects,
// arrays, doubles, strings without \u escapes, true/false/null -- and refuses
// anything else with a message rather than guessing.
//
// Schemas it has to read:
//   * the .silw sidecar   {layer_sizes, activation, obs_dim, act_dim,
//                          normalizer:{mean,std}}
//   * the calibration file [{bus_id, zero_steps, dir}, ...]
//   * Agent B's golden vectors {cases:[{obs:[...], action:[...]}, ...]}
#pragma once
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <vector>

namespace siljson {

struct Value {
    enum Type { kNull, kBool, kNum, kStr, kArr, kObj };
    Type type = kNull;
    bool b = false;
    double num = 0.0;
    std::string str;
    std::vector<Value> arr;          // kArr
    std::vector<std::string> keys;   // kObj, parallel to vals
    std::vector<Value> vals;

    bool isNum() const { return type == kNum; }
    bool isArr() const { return type == kArr; }
    bool isObj() const { return type == kObj; }
    bool isStr() const { return type == kStr; }

    // Member lookup; nullptr when absent or when this is not an object.
    const Value* get(const char* key) const {
        if (type != kObj) return nullptr;
        for (size_t i = 0; i < keys.size(); ++i) {
            if (keys[i] == key) return &vals[i];
        }
        return nullptr;
    }
    // First of several spellings that is present -- the exporter and the
    // golden generator are written by another agent, so a little tolerance
    // here is cheaper than a broken integration.
    const Value* getAny(const char* const* names, size_t n) const {
        for (size_t i = 0; i < n; ++i) {
            if (const Value* v = get(names[i])) return v;
        }
        return nullptr;
    }

    bool asFloats(std::vector<float>& out) const {
        if (type != kArr) return false;
        out.clear();
        out.reserve(arr.size());
        for (const Value& v : arr) {
            if (v.type != kNum) return false;
            out.push_back(static_cast<float>(v.num));
        }
        return true;
    }
    bool asInts(std::vector<int>& out) const {
        if (type != kArr) return false;
        out.clear();
        out.reserve(arr.size());
        for (const Value& v : arr) {
            if (v.type != kNum) return false;
            out.push_back(static_cast<int>(lrint(v.num)));
        }
        return true;
    }
};

namespace detail {

struct Cursor {
    const char* s;
    size_t n;
    size_t i;
    std::string err;
};

inline bool fail(Cursor& c, const char* what) {
    if (c.err.empty()) {
        char buf[128];
        snprintf(buf, sizeof buf, "%s at byte %zu", what, c.i);
        c.err = buf;
    }
    return false;
}

inline void skipWs(Cursor& c) {
    while (c.i < c.n) {
        const char ch = c.s[c.i];
        if (ch == ' ' || ch == '\t' || ch == '\n' || ch == '\r') {
            ++c.i;
        } else {
            break;
        }
    }
}

bool parseValue(Cursor& c, Value& out, int depth);

inline bool parseString(Cursor& c, std::string& out) {
    if (c.i >= c.n || c.s[c.i] != '"') return fail(c, "expected string");
    ++c.i;
    out.clear();
    while (c.i < c.n) {
        const char ch = c.s[c.i++];
        if (ch == '"') return true;
        if (ch != '\\') {
            out.push_back(ch);
            continue;
        }
        if (c.i >= c.n) return fail(c, "truncated escape");
        const char e = c.s[c.i++];
        switch (e) {
            case '"': out.push_back('"'); break;
            case '\\': out.push_back('\\'); break;
            case '/': out.push_back('/'); break;
            case 'b': out.push_back('\b'); break;
            case 'f': out.push_back('\f'); break;
            case 'n': out.push_back('\n'); break;
            case 'r': out.push_back('\r'); break;
            case 't': out.push_back('\t'); break;
            // \uXXXX is not needed by any schema here, and silently mangling
            // it would be worse than refusing the file.
            default: return fail(c, "unsupported string escape");
        }
    }
    return fail(c, "unterminated string");
}

inline bool parseNumber(Cursor& c, Value& out) {
    const char* start = c.s + c.i;
    char* end = nullptr;
    const double v = strtod(start, &end);
    if (end == start) return fail(c, "expected number");
    c.i += static_cast<size_t>(end - start);
    out.type = Value::kNum;
    out.num = v;
    return true;
}

inline bool lit(Cursor& c, const char* word) {
    const size_t len = strlen(word);
    if (c.i + len > c.n || memcmp(c.s + c.i, word, len) != 0) return false;
    c.i += len;
    return true;
}

inline bool parseValue(Cursor& c, Value& out, int depth) {
    if (depth > 32) return fail(c, "nesting too deep");
    skipWs(c);
    if (c.i >= c.n) return fail(c, "unexpected end of input");
    const char ch = c.s[c.i];
    if (ch == '{') {
        ++c.i;
        out.type = Value::kObj;
        skipWs(c);
        if (c.i < c.n && c.s[c.i] == '}') { ++c.i; return true; }
        for (;;) {
            skipWs(c);
            std::string key;
            if (!parseString(c, key)) return false;
            skipWs(c);
            if (c.i >= c.n || c.s[c.i] != ':') return fail(c, "expected ':'");
            ++c.i;
            Value v;
            if (!parseValue(c, v, depth + 1)) return false;
            out.keys.push_back(key);
            out.vals.push_back(v);
            skipWs(c);
            if (c.i < c.n && c.s[c.i] == ',') { ++c.i; continue; }
            if (c.i < c.n && c.s[c.i] == '}') { ++c.i; return true; }
            return fail(c, "expected ',' or '}'");
        }
    }
    if (ch == '[') {
        ++c.i;
        out.type = Value::kArr;
        skipWs(c);
        if (c.i < c.n && c.s[c.i] == ']') { ++c.i; return true; }
        for (;;) {
            Value v;
            if (!parseValue(c, v, depth + 1)) return false;
            out.arr.push_back(v);
            skipWs(c);
            if (c.i < c.n && c.s[c.i] == ',') { ++c.i; continue; }
            if (c.i < c.n && c.s[c.i] == ']') { ++c.i; return true; }
            return fail(c, "expected ',' or ']'");
        }
    }
    if (ch == '"') {
        out.type = Value::kStr;
        return parseString(c, out.str);
    }
    if (lit(c, "true")) { out.type = Value::kBool; out.b = true; return true; }
    if (lit(c, "false")) { out.type = Value::kBool; out.b = false; return true; }
    if (lit(c, "null")) { out.type = Value::kNull; return true; }
    return parseNumber(c, out);
}

}  // namespace detail

inline bool parse(const std::string& text, Value& out, std::string& err) {
    detail::Cursor c{text.c_str(), text.size(), 0, std::string()};
    if (!detail::parseValue(c, out, 0)) {
        err = c.err;
        return false;
    }
    detail::skipWs(c);
    if (c.i != c.n) {
        err = "trailing bytes after the top-level JSON value";
        return false;
    }
    return true;
}

inline bool readFile(const char* path, std::string& out, std::string& err) {
    FILE* f = fopen(path, "rb");
    if (!f) {
        err = std::string("cannot open ") + path;
        return false;
    }
    out.clear();
    char buf[4096];
    size_t got = 0;
    while ((got = fread(buf, 1, sizeof buf, f)) > 0) out.append(buf, got);
    const bool bad = ferror(f) != 0;
    fclose(f);
    if (bad) {
        err = std::string("read error on ") + path;
        return false;
    }
    return true;
}

inline bool parseFile(const char* path, Value& out, std::string& err) {
    std::string text;
    if (!readFile(path, text, err)) return false;
    if (!parse(text, out, err)) {
        err = std::string(path) + ": " + err;
        return false;
    }
    return true;
}

inline bool fileExists(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    fclose(f);
    return true;
}

}  // namespace siljson
