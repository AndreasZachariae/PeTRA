#include <petra_core/tools/Logger.h>

#include <petra_core/tools/IO.h>

Logger Logger::global_instance = Logger();

void Logger::log(const std::string &message, unsigned int hierarchy, LogLevel level)
{
    LogEntry entry = LogEntry();

    entry.message = message;
    entry.hierarchy = hierarchy;
    entry.level = level;

    entries.push_back(entry);

    write_log(entry);
}

void Logger::write_log(const LogEntry &entry)
{
    if (entry.level >= this->level)
    {
        std::string message = "[" + entry.level.to_string() + " | " + entry.time.to_string("%H:%M:%S") + "]: " + entry.message;

        //im log.txt wird immer nur eine Zeile mit dem neuesten Eintrag überschrieben
        //IO::write_file(file_path, message, true); //temporary disabled

        for (size_t i = 0; i < entry.hierarchy; ++i)
        {
            std::cout << HIERARCHY_PREFIX;
        }

        std::cout << message << std::endl;
    }
}