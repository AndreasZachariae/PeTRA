#pragma once

#include <string>
#include <map>
#include <chrono>

class ChronoTime
{
private:
  std::chrono::system_clock::time_point time_;

public:
  ChronoTime();
  ~ChronoTime();
  
  int64_t duration();
  int64_t duration(const ChronoTime &until);

  std::string to_string(const std::string& format) const;
};
