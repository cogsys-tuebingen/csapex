/// HEADER
#include <csapex/scheduling/thread_group.h>

/// PROJECT
#include <csapex/scheduling/task.h>
#include <csapex/utility/thread.h>
#include <csapex/scheduling/task_generator.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

int ThreadGroup::next_id_ = ThreadGroup::MINIMUM_THREAD_ID;

ThreadGroup::ThreadGroup(int id, std::string name)
    : id_(id), name_(name), running_(false), pause_(false)
{
    next_id_ = std::max(next_id_, id + 1);
    startThread();
}
ThreadGroup::ThreadGroup(std::string name)
    : id_(next_id_++), name_(name), running_(false), pause_(false)
{
    startThread();
}

ThreadGroup::~ThreadGroup()
{
    if(running_ || thread_.joinable()) {
        stop();
    }
}

int ThreadGroup::nextId()
{
    return next_id_;
}

int ThreadGroup::id() const
{
    return id_;
}

std::string ThreadGroup::name() const
{
    return name_;
}
const std::thread& ThreadGroup::thread() const
{
    return thread_;
}

bool ThreadGroup::isEmpty() const
{
    return generators_.empty();
}



void ThreadGroup::setPause(bool pause)
{
    if(pause != pause_) {
        pause_ = pause;

        std::unique_lock<std::recursive_mutex> lock(state_mtx_);
        pause_changed_.notify_all();
    }
}

void ThreadGroup::stop()
{
    {
        std::unique_lock<std::recursive_mutex> lock(state_mtx_);
        running_ = false;
        pause_ = false;
        pause_changed_.notify_all();
    }
    {
        std::unique_lock<std::recursive_mutex> lock(tasks_mtx_);
        work_available_.notify_all();
        if(thread_.joinable()) {
            lock.unlock();

            thread_.join();
        }
    }
}

void ThreadGroup::add(TaskGenerator* generator)
{
    std::unique_lock<std::recursive_mutex> lock(state_mtx_);
    if(!running_) {
        startThread();
    }
    generators_.push_back(generator);
}

void ThreadGroup::add(TaskGenerator *generator, const std::vector<TaskPtr> &initial_tasks)
{
    add(generator);

    std::unique_lock<std::recursive_mutex> lock(tasks_mtx_);
    for(TaskPtr t: initial_tasks) {
        schedule(t);
    }

    work_available_.notify_all();
}

std::vector<TaskPtr> ThreadGroup::remove(TaskGenerator* generator)
{
    std::vector<TaskPtr> remaining_tasks;

    std::unique_lock<std::recursive_mutex> lock(tasks_mtx_);

    for(auto it = tasks_.begin(); it != tasks_.end();) {
        TaskPtr task = *it;
        if(task->getParent() == generator) {
            remaining_tasks.push_back(task);
            it = tasks_.erase(it);
        } else {
            ++it;
        }
    }

    for(auto it = generators_.begin(); it != generators_.end();) {
        if(*it == generator) {
            it = generators_.erase(it);
        } else {
            ++it;
        }
    }

    work_available_.notify_all();

    return remaining_tasks;
}

void ThreadGroup::schedule(TaskPtr task)
{
    std::unique_lock<std::recursive_mutex> tasks_lock(tasks_mtx_);

    for(TaskPtr t : tasks_) {
        if(t.get() == task.get()) {
            return;
        }
    }

    tasks_.push_back(task);

    work_available_.notify_all();
}

void ThreadGroup::startThread()
{
    std::unique_lock<std::recursive_mutex> lock(state_mtx_);
    if(thread_.joinable()) {
        running_ = false;
        pause_changed_.notify_all();
        lock.unlock();

        thread_.join();
    }

    running_ = true;

    thread_ = std::thread ([this]() {

        csapex::thread::set_name((name_ + "!").c_str());

        while(running_) {
            {
                std::unique_lock<std::recursive_mutex> lock(tasks_mtx_);
                running_ = !generators_.empty();
                while(running_ && tasks_.empty()) {
                    work_available_.wait_for(lock, std::chrono::seconds(1));
                    running_ = running_ && !generators_.empty();
                }
            }

            bool done = !running_;
            while(!done) {
                {
                    // pause?
                    std::unique_lock<std::recursive_mutex> state_lock(state_mtx_);
                    while(running_ && pause_) {
                        pause_changed_.wait(state_lock);
                    }
                }

                // execute one task
                std::unique_lock<std::recursive_mutex> tasks_lock(tasks_mtx_);
                if(tasks_.empty()) {
                    done = true;

                } else {
                    auto task = tasks_.front();
                    tasks_.pop_front();

                    tasks_lock.unlock();

                    {
                        std::unique_lock<std::recursive_mutex> state_lock(state_mtx_);
                        if(running_) {
                            state_lock.unlock();

                            try {
                                task->execute();

                            } catch(const std::exception& e) {
                                TaskGenerator* gen = task->getParent();
                                gen->setError(e.what());

                            } catch(const std::string& s) {
                                std::cerr << "Uncatched exception (string) exception: " << s << std::endl;
                            } catch(...) {
                                std::cerr << "Uncatched exception of unknown type and origin!" << std::endl;
                                std::abort();
                            }

                        } else {
                            done = true;
                        }
                    }
                }
            }
        }
    });
}
