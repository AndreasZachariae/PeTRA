#pragma once

#include <cmath>

#include <petra_core/default.h>

#include <petra_core/tools/Chrono.h>
#include <petra_core/tools/Logger.h>

class Component
{
public:
  Component() : Component("Unknown") {}
  Component(const std::string &name) : Component(name, "", 0) {}
  Component(const std::string &name, const std::string &description) : Component(name, description, 0) {}
  Component(const std::string &name, unsigned int hierarchy) : Component(name, "", hierarchy) {}
  Component(const std::string &name, const std::string &description, unsigned int hierarchy) : name_(name), description_(description), hierarchy_(hierarchy) {}

  void log(const std::string &message, LogLevel level) { Logger::global_instance.log(prefixed(message), hierarchy_, level); }
  void log(const std::string &message) { log(message, LogLevel::Info); }

  void start_chrono(const std::string &key) { Chrono::global_instance.start(prefixed(key)); }
  int64_t stop_chrono(const std::string &key) { return Chrono::global_instance.stop(prefixed(key)); }
  void log_chrono(const std::string &key) { log(key + ": " + std::to_string(Chrono::global_instance.stop(prefixed(key)) / 1e6) + " ms"); }

  std::string get_name() { return name_; }

  std::string ftos(float float_value) { return ftos(std::to_string(std::roundf(float_value * 100.0) / 100.0)); }
  std::string ftos(std::string float_str)
  {
    float_str.erase(float_str.find_last_not_of('0') + 1, std::string::npos);
    float_str.erase(float_str.find_last_not_of('.') + 1, std::string::npos);
    return float_str;
  }

protected:
  std::string name_;
  std::string description_;

private:
  unsigned int hierarchy_;

  std::string prefixed(const std::string &value) { return "[" + name_ + "] " + value; }
};