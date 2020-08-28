#pragma once

#include <petra_core/default.h>

#include <thread>

class ThreadExecuter
{
public:
    ThreadExecuter(const std::string &filepath) : filepath_(filepath) {}
    ~ThreadExecuter() {}

    std::string filepath() { return filepath_; }
    bool is_running() { return thread_handle_ > 0; }

    bool run();
    bool cancel();

private:
    std::string filepath_;

    pthread_t thread_handle_ = 0;
};