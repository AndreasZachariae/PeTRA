#pragma once

#include <string>

class Event
{
public:
  Event(const std::string &type, const std::string &message) : type(type), message(message), id_(++max_id_) {}
  Event(const std::string &type) : Event(type, "") {}

  std::string type;
  std::string message;

  int id() { return id_; }
  
private:
  static int max_id_;

  int id_;
};