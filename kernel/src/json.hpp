// Minimal JSON value, parser and writer for the kernel binding.
// Deliberately tiny: the kernel only needs the protocol's message shapes.
// SPDX-License-Identifier: LGPL-2.1-only
#pragma once

#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace cadoc::json {

struct Value {
  enum class Kind { Null, Bool, Number, String, Array, Object } kind = Kind::Null;
  bool b = false;
  double n = 0;
  std::string s;
  std::vector<Value> a;
  std::map<std::string, Value> o;

  bool has(const std::string& k) const { return kind == Kind::Object && o.count(k) > 0; }
  const Value& operator[](const std::string& k) const {
    auto it = o.find(k);
    if (kind != Kind::Object || it == o.end()) throw std::runtime_error("missing field: " + k);
    return it->second;
  }
  /// A number, or a non-finite number in its wire form {"$float": "Infinity" | "-Infinity" | "NaN"}.
  double num() const {
    if (kind == Kind::Object && o.size() == 1 && o.count("$float") && o.at("$float").kind == Kind::String) {
      const std::string& f = o.at("$float").s;
      if (f == "Infinity") return INFINITY;
      if (f == "-Infinity") return -INFINITY;
      if (f == "NaN") return NAN;
      throw std::runtime_error("bad $float value " + f);
    }
    if (kind != Kind::Number) throw std::runtime_error("expected number");
    return n;
  }
  bool isNull() const { return kind == Kind::Null; }
  /// Optional fields: absent or null gives the default.
  const Value* get(const std::string& k) const {
    if (kind != Kind::Object) return nullptr;
    auto it = o.find(k);
    return it == o.end() || it->second.kind == Kind::Null ? nullptr : &it->second;
  }
  const std::string& str() const {
    if (kind != Kind::String) throw std::runtime_error("expected string");
    return s;
  }
  bool boolean() const {
    if (kind != Kind::Bool) throw std::runtime_error("expected bool");
    return b;
  }
  const std::vector<Value>& arr() const {
    if (kind != Kind::Array) throw std::runtime_error("expected array");
    return a;
  }
};

class Parser {
 public:
  explicit Parser(const std::string& text) : t_(text) {}
  Value parse() {
    Value v = value();
    ws();
    if (i_ != t_.size()) fail("trailing characters");
    return v;
  }

 private:
  const std::string& t_;
  size_t i_ = 0;

  [[noreturn]] void fail(const char* what) {
    throw std::runtime_error(std::string("json: ") + what + " at " + std::to_string(i_));
  }
  void ws() {
    while (i_ < t_.size() && (t_[i_] == ' ' || t_[i_] == '\n' || t_[i_] == '\r' || t_[i_] == '\t')) ++i_;
  }
  bool eat(char c) {
    ws();
    if (i_ < t_.size() && t_[i_] == c) { ++i_; return true; }
    return false;
  }
  Value value() {
    ws();
    if (i_ >= t_.size()) fail("unexpected end");
    char c = t_[i_];
    Value v;
    if (c == '{') {
      ++i_;
      v.kind = Value::Kind::Object;
      if (eat('}')) return v;
      do {
        ws();
        std::string k = string();
        if (!eat(':')) fail("expected ':'");
        v.o[k] = value();
      } while (eat(','));
      if (!eat('}')) fail("expected '}'");
    } else if (c == '[') {
      ++i_;
      v.kind = Value::Kind::Array;
      if (eat(']')) return v;
      do { v.a.push_back(value()); } while (eat(','));
      if (!eat(']')) fail("expected ']'");
    } else if (c == '"') {
      v.kind = Value::Kind::String;
      v.s = string();
    } else if (t_.compare(i_, 4, "true") == 0) {
      i_ += 4; v.kind = Value::Kind::Bool; v.b = true;
    } else if (t_.compare(i_, 5, "false") == 0) {
      i_ += 5; v.kind = Value::Kind::Bool; v.b = false;
    } else if (t_.compare(i_, 4, "null") == 0) {
      i_ += 4;
    } else {
      size_t end = i_;
      while (end < t_.size() && (isdigit((unsigned char)t_[end]) || t_[end] == '-' || t_[end] == '+' ||
                                 t_[end] == '.' || t_[end] == 'e' || t_[end] == 'E'))
        ++end;
      if (end == i_) fail("unexpected character");
      v.kind = Value::Kind::Number;
      v.n = std::stod(t_.substr(i_, end - i_));
      i_ = end;
    }
    return v;
  }
  std::string string() {
    if (i_ >= t_.size() || t_[i_] != '"') fail("expected string");
    ++i_;
    std::string out;
    while (i_ < t_.size() && t_[i_] != '"') {
      char c = t_[i_++];
      if (c != '\\') { out += c; continue; }
      if (i_ >= t_.size()) fail("bad escape");
      char e = t_[i_++];
      switch (e) {
        case 'n': out += '\n'; break;
        case 't': out += '\t'; break;
        case 'r': out += '\r'; break;
        case 'b': out += '\b'; break;
        case 'f': out += '\f'; break;
        case 'u': {
          unsigned cp = std::stoul(t_.substr(i_, 4), nullptr, 16);
          i_ += 4;
          if (cp < 0x80) out += char(cp);
          else if (cp < 0x800) { out += char(0xC0 | (cp >> 6)); out += char(0x80 | (cp & 0x3F)); }
          else { out += char(0xE0 | (cp >> 12)); out += char(0x80 | ((cp >> 6) & 0x3F)); out += char(0x80 | (cp & 0x3F)); }
          break;
        }
        default: out += e;
      }
    }
    if (i_ >= t_.size()) fail("unterminated string");
    ++i_;
    return out;
  }
};

/// Streaming writer: the caller emits tokens in order; commas are automatic.
class Writer {
 public:
  std::string out;

  Writer& beginObject() { sep(); out += '{'; first_.push_back(true); return *this; }
  Writer& endObject() { out += '}'; first_.pop_back(); return *this; }
  Writer& beginArray() { sep(); out += '['; first_.push_back(true); return *this; }
  Writer& endArray() { out += ']'; first_.pop_back(); return *this; }
  Writer& key(const char* k) { sep(); quote(k); out += ':'; afterKey_ = true; return *this; }
  Writer& str(const std::string& s) { sep(); quote(s); return *this; }
  Writer& boolean(bool v) { sep(); out += v ? "true" : "false"; return *this; }
  Writer& integer(long long v) { sep(); out += std::to_string(v); return *this; }
  /// Shortest round-trip form (the same digits as JSON.stringify and Python's repr).
  /// Non-finite numbers use the protocol's wire form {"$float": "Infinity"}.
  Writer& num(double v) {
    if (!std::isfinite(v)) {
      beginObject().key("$float").str(std::isnan(v) ? "NaN" : v > 0 ? "Infinity" : "-Infinity");
      return endObject();
    }
    sep();
    if (v == 0) v = 0;  // no "-0"
    char buf[40];
    auto r = std::to_chars(buf, buf + sizeof buf, v);
    out.append(buf, r.ptr);
    return *this;
  }
  /// Splices already-serialised JSON in as one value.
  Writer& raw(const std::string& json) { sep(); out += json; return *this; }

 private:
  std::vector<bool> first_;
  bool afterKey_ = false;

  void sep() {
    if (afterKey_) { afterKey_ = false; return; }
    if (!first_.empty()) {
      if (!first_.back()) out += ',';
      first_.back() = false;
    }
  }
  void quote(const std::string& s) {
    out += '"';
    for (unsigned char c : s) {
      switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
          if (c < 0x20) { char b[8]; std::snprintf(b, sizeof b, "\\u%04x", c); out += b; }
          else out += char(c);
      }
    }
    out += '"';
  }
};

}  // namespace cadoc::json
