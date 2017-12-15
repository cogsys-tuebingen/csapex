/// HEADER
#include <csapex/scheduling/timed_queue.h>

/// COMPONENT
#include <csapex/scheduling/scheduler.h>
#include <csapex/utility/thread.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

TimedQueue::TimedQueue()
    : scheduling_running_(false), sleeping_running_(false)
{
    next_wake_up_ = std::chrono::system_clock::now();
    start();
}
TimedQueue::~TimedQueue()
{
    stop();
}

void TimedQueue::start()
{
    // this thread schedules tasks that have expired
    scheduling_running_ = true;
    scheduling_thread_ = std::thread([this](){
        loop();
    });

    // this thread sleeps until the time given by 'next_wake_up_' is reached.
    // it then wakes up the scheduling thread
    sleeping_running_ = true;
    sleeping_thread_ = std::thread([this](){
        sleep();
    });
}

void TimedQueue::stop()
{
    if(scheduling_thread_.joinable()) {
        {
            std::unique_lock<std::mutex> lock(task_mtx_);
            scheduling_running_ = false;
        }
        apex_assert_eq_hard(false, scheduling_running_);
        tasks_changed_.notify_all();
        apex_assert_eq_hard(false, scheduling_running_);
        scheduling_thread_.join();
    }
    if(sleeping_thread_.joinable()) {
        {
            std::unique_lock<std::mutex> lock(sleep_mtx_);
            sleeping_running_ = false;
        }
        apex_assert_eq_hard(false, sleeping_running_);
        next_wake_up_changed_.notify_all();
        apex_assert_eq_hard(false, sleeping_running_);
        sleeping_thread_.join();
    }
}

void TimedQueue::loop()
{
    csapex::thread::set_name("queue:exec");

    std::unique_lock<std::mutex> lock(task_mtx_);
    while(scheduling_running_) {
        // wait until a task is available
        while(tasks_.empty()) {
            tasks_changed_.wait_for(lock, std::chrono::milliseconds(100));
            if(!scheduling_running_) {
                return;
            }
        }

        // schedule all tasks that have expired
        while(!tasks_.empty()) {
            const Unit& front = *tasks_.begin();

            auto now = std::chrono::system_clock::now();
            if(now >= front.time) {
                Unit unit = front;
                tasks_.erase(tasks_.begin());
                lock.unlock();


                unit.scheduler->schedule(unit.schedulable);

                lock.lock();
            } else {
                break;
            }
        }

        // notify the sleeping thread, if there are remaining tasks
        if(!tasks_.empty()) {
            Unit front = *tasks_.begin();
            lock.unlock();

            {
                std::unique_lock<std::mutex> lock(sleep_mtx_);
                next_wake_up_ = front.time;
                next_wake_up_changed_.notify_all();
            }

            lock.lock();
        }
    }
}

void TimedQueue::sleep()
{
    csapex::thread::set_name("queue:sleep");

    std::unique_lock<std::mutex> lock(sleep_mtx_);
    while(sleeping_running_) {
        // wait for a sleep command
        next_wake_up_changed_.wait(lock);

        auto now = std::chrono::system_clock::now();
        if(now < next_wake_up_) {
            // the next sleep target is in the future -> sleep
            std::this_thread::sleep_until(next_wake_up_);
            tasks_changed_.notify_all();
        }
    }
}

void TimedQueue::schedule(SchedulerPtr scheduler, TaskPtr schedulable, std::chrono::system_clock::time_point time)
{
    Unit unit;
    unit.scheduler = scheduler;
    unit.schedulable = schedulable;
    unit.time = time;

    std::unique_lock<std::mutex> lock(task_mtx_);
    auto front = tasks_.begin();
    tasks_.insert(unit);

    if(tasks_.begin() != front) {
        // the earliest entry changed -> notify the scheduling thread immediately
        tasks_changed_.notify_all();
    }
}
