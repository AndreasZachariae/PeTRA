#include <petra_central_control/SystemMonitor.h>

void SystemMonitor::spin_()
{
    check_system();
}

void SystemMonitor::check_system()
{
    //log("Background system check...");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}