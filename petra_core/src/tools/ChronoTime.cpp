#include <petra_core/tools/ChronoTime.h>

#include <iomanip>
#include <sstream>

ChronoTime::ChronoTime()
{
    time_ = std::chrono::system_clock::now();
}

ChronoTime::~ChronoTime()
{
}

int64_t ChronoTime::duration()
{
    return duration(ChronoTime());
}

int64_t ChronoTime::duration(const ChronoTime &until)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(time_ - until.time_).count();
}

std::string ChronoTime::to_string(const std::string &format) const
{
    std::time_t tt = std::chrono::system_clock::to_time_t(time_);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&tt), format.c_str());

    return ss.str();
}