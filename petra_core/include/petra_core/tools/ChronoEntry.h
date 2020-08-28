#pragma once

#include <chrono>

#include <petra_core/tools/ChronoTime.h>

class ChronoEntry
{
public:
    ChronoEntry() {}
    ~ChronoEntry() {}

    void start() { start_time_ = ChronoTime(); }
    void stop() { stop_time_ = ChronoTime(); }
    int64_t duration() { return start_time_.duration(stop_time_); }

private:
    ChronoTime start_time_;
    ChronoTime stop_time_;
};