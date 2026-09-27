// Minimal JSON reader (objects, arrays, numbers, strings, bool, null).
// Vendored to keep the build free of non-ROS dependencies (offline colcon build).
#pragma once

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace tram_odometry::json
{

struct Value
{
  enum class Type { Null, Bool, Number, String, Array, Object };
  Type type{Type::Null};
  bool b{false};
  double num{0.0};
  std::string str;
  std::vector<Value> arr;
  std::map<std::string, Value> obj;

  bool has(const std::string & k) const { return type == Type::Object && obj.count(k) > 0; }
  const Value & at(const std::string & k) const
  {
    auto it = obj.find(k);
    if (type != Type::Object || it == obj.end()) {
      throw std::runtime_error("json: missing key '" + k + "'");
    }
    return it->second;
  }
  double number() const
  {
    if (type != Type::Number) {throw std::runtime_error("json: not a number");}
    return num;
  }
};

Value parse(const std::string & text);
Value parse_file(const std::string & path);

}  // namespace tram_odometry::json
