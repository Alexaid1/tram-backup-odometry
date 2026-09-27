#include "tram_odometry/core/json_lite.hpp"

#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace tram_odometry::json
{
namespace
{

class Parser
{
public:
  explicit Parser(const std::string & s)
  : s_(s) {}

  Value parse_root()
  {
    Value v = value();
    ws();
    if (i_ != s_.size()) {fail("trailing characters");}
    return v;
  }

private:
  const std::string & s_;
  size_t i_{0};

  [[noreturn]] void fail(const char * what) const
  {
    throw std::runtime_error(std::string("json: ") + what + " at offset " + std::to_string(i_));
  }
  void ws()
  {
    while (i_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[i_]))) {++i_;}
  }
  char peek()
  {
    ws();
    if (i_ >= s_.size()) {fail("unexpected end");}
    return s_[i_];
  }
  void expect(char c)
  {
    if (peek() != c) {fail("unexpected character");}
    ++i_;
  }
  bool literal(const char * lit)
  {
    size_t n = std::char_traits<char>::length(lit);
    if (s_.compare(i_, n, lit) == 0) {i_ += n; return true;}
    return false;
  }

  Value value()
  {
    char c = peek();
    Value v;
    if (c == '{') {
      v.type = Value::Type::Object;
      ++i_;
      if (peek() == '}') {++i_; return v;}
      while (true) {
        std::string k = string_raw();
        expect(':');
        v.obj.emplace(std::move(k), value());
        char d = peek();
        ++i_;
        if (d == '}') {break;}
        if (d != ',') {fail("expected , or }");}
      }
    } else if (c == '[') {
      v.type = Value::Type::Array;
      ++i_;
      if (peek() == ']') {++i_; return v;}
      while (true) {
        v.arr.push_back(value());
        char d = peek();
        ++i_;
        if (d == ']') {break;}
        if (d != ',') {fail("expected , or ]");}
      }
    } else if (c == '"') {
      v.type = Value::Type::String;
      v.str = string_raw();
    } else if (literal("true")) {
      v.type = Value::Type::Bool; v.b = true;
    } else if (literal("false")) {
      v.type = Value::Type::Bool; v.b = false;
    } else if (literal("null")) {
      v.type = Value::Type::Null;
    } else {
      const char * begin = s_.c_str() + i_;
      char * end = nullptr;
      v.num = std::strtod(begin, &end);
      if (end == begin) {fail("bad value");}
      v.type = Value::Type::Number;
      i_ += static_cast<size_t>(end - begin);
    }
    return v;
  }

  std::string string_raw()
  {
    expect('"');
    std::string out;
    while (i_ < s_.size() && s_[i_] != '"') {
      if (s_[i_] == '\\' && i_ + 1 < s_.size()) {
        char e = s_[i_ + 1];
        if (e == 'u') {  // keep \uXXXX escapes verbatim (only used in names)
          out += s_.substr(i_, 6);
          i_ += 6;
          continue;
        }
        out += (e == 'n') ? '\n' : (e == 't') ? '\t' : e;
        i_ += 2;
      } else {
        out += s_[i_++];
      }
    }
    if (i_ >= s_.size()) {fail("unterminated string");}
    ++i_;
    return out;
  }
};

}  // namespace

Value parse(const std::string & text)
{
  return Parser(text).parse_root();
}

Value parse_file(const std::string & path)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {throw std::runtime_error("json: cannot open " + path);}
  std::stringstream ss;
  ss << f.rdbuf();
  return parse(ss.str());
}

}  // namespace tram_odometry::json
