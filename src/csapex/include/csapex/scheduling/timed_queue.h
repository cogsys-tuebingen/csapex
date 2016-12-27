#ifndef TIMED_QUEUE_H
#define TIMED_QUEUE_H

/// COMPONENT
#include <csapex/scheduling/scheduling_fwd.h>

/// SYSTEM
#include <thread>
#include <condition_variable>
#include <chrono>
#include <set>

namespace csapex
{

class TimedQueue
{
public:
    TimedQueue();
    ~TimedQueue();

    void start();
    void stop();

    void schedule(SchedulerPtr scheduler, TaskPtr schedulable, std::chrono::system_clock::time_point time);

private:
    void loop();
    void sleep();

private:
    std::thread scheduling_thread_;
    std::thread sleeping_thread_;

    bool scheduling_running_;
    bool sleeping_running_;

    struct Unit {
        SchedulerPtr scheduler;
        TaskPtr schedulable;
        std::chrono::system_clock::time_point time;
    };

    struct UnitCompare
    {
        bool operator() (const Unit& a, const Unit& b) const
        {
            return a.time < b.time;
        }
    };

    std::mutex task_mtx_;
    std::condition_variable tasks_changed_;
    std::set<Unit, UnitCompare> tasks_;

    std::mutex sleep_mtx_;
    std::condition_variable next_wake_up_changed_;
    std::chrono::system_clock::time_point next_wake_up_;
};

}

#endif // TIMED_QUEUE_H
