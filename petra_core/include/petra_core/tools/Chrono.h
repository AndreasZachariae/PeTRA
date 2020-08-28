#pragma once

#include <petra_core/default.h>

#include <petra_core/tools/ChronoEntry.h>

class Chrono
{
public:
    static Chrono global_instance;

    Chrono() {}
    ~Chrono() {}

    std::string start();
    std::string start(std::string key);
    int64_t stop(std::string key);

private:
    std::map<std::string, ChronoEntry> entries;
};