#pragma once

#include <petra_core/default.h>

#include <iostream>

#include <petra_core/tools/LogEntry.h>

class Logger
{
public:
    const std::string HIERARCHY_PREFIX = "  ";
    static Logger global_instance;

    Logger() : Logger(LogLevel::Info) {}
    Logger(LogLevel level) : level(level) {}
    ~Logger() {}

    LogLevel level;
    std::string file_path = "log.txt";

    void log(const std::string &message, unsigned int hierarchy, LogLevel level);
    void log(const std::string &message, unsigned int hierarchy) { log(message, hierarchy, LogLevel::Info); }
    void log(const std::string &message) { log(message, 0); }

private:
    std::vector<LogEntry> entries;

    void write_log(const LogEntry &entry);
};