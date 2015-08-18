/// HEADER
#include <csapex/scheduling/thread_group.h>

/// PROJECT
#include <csapex/scheduling/task.h>
#include <csapex/utility/thread.h>
#include <csapex/scheduling/task_generator.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

int ThreadGroup::next_id_ = ThreadGroup::MINIMUM_THREAD_ID;

ThreadGroup::ThreadGroup(int id, std::string name, bool paused)
    : id_(id), name_(name), running_(false), pause_(paused)
{
    next_id_ = std::max(next_id_, id + 1);
    startThread();
}
ThreadGroup::ThreadGroup(std::string name, bool paused)
    : id_(next_id_++), name_(name), running_(false), pause_(paused)
{
    startThread();
}

ThreadGroup::~ThreadGroup()
{
    if(running_ || scheduler_thread_.joinable()) {
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
    return scheduler_thread_;
}

bool ThreadGroup::isEmpty() const
{
    return generators_.empty();
}



void ThreadGroup::setPause(bool pause)
{
    if(pause != pause_) {
        pause_ = pause;

        for(auto generator : generators_) {
            generator->setPause(pause);
        }

        std::unique_lock<std::recursive_mutex> lock(state_mtx_);
        pause_changed_.notify_all();
    }
}

void ThreadGroup::setSteppingMode(bool stepping)
{
    std::unique_lock<std::recursive_mutex> state_lock(execution_mtx_);
    for(auto generator : generators_) {
        generator->setSteppingMode(stepping);
    }
}

void ThreadGroup::step()
{
    begin_step();

    std::unique_lock<std::recursive_mutex> state_lock(execution_mtx_);
    for(auto generator : generators_) {
        generator->step();
    }
}

bool ThreadGroup::isStepping() const
{
    // this is only consistency checking...
    int stepping = 0;
    int not_stepping = 0;
    for(auto generator : generators_) {
        if(generator->isStepping()) {
            ++stepping;
        } else {
            ++not_stepping;
        }
    }

    apex_assert_hard(stepping == 0 || not_stepping == 0);

    return stepping > 0;
}

bool ThreadGroup::isStepDone() const
{
    for(auto generator : generators_) {
        if(!generator->isStepDone()) {
            return false;
        }
    }

    return true;
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
        std::unique_lock<std::recursive_mutex> state_lock(execution_mtx_);
    }
    {
        std::unique_lock<std::recursive_mutex> lock(tasks_mtx_);
        work_available_.notify_all();
        if(scheduler_thread_.joinable()) {
            lock.unlock();

            scheduler_thread_.join();
        }
    }
}

void ThreadGroup::clear()
{

    std::unique_lock<std::recursive_mutex> state_lock(execution_mtx_);
    {
        std::unique_lock<std::recursive_mutex> lock(tasks_mtx_);
        tasks_.clear();
    }
    for(auto generator : generators_) {
        generator->reset();
    }
}

void ThreadGroup::add(TaskGenerator* generator)
{
    generator->setPause(pause_);

    std::unique_lock<std::recursive_mutex> lock(state_mtx_);
    if(!running_) {
        startThread();
    }
    generators_.push_back(generator);

    auto& cs = generator_connections_[generator];

    cs.push_back(generator->end_step.connect([this]() { checkIfStepIsDone(); }));
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

void ThreadGroup::checkIfStepIsDone()
{
    if(isStepDone()) {
        end_step();
    }
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

    for(auto connection : generator_connections_[generator]) {
        connection.disconnect();
    }

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
    if(scheduler_thread_.joinable()) {
        running_ = false;
        pause_changed_.notify_all();
        lock.unlock();

        scheduler_thread_.join();
    }

    running_ = true;

    scheduler_thread_ = std::thread ([this]() {
        csapex::thread::set_name((name_).c_str());
        schedulingLoop();
    });
}

void ThreadGroup::schedulingLoop()
{
    while(running_) {
        bool keep_executing = waitForTasks();
        while(running_ && keep_executing) {
            handlePause();

            keep_executing = executeNextTask();
        }
    }
}

bool ThreadGroup::waitForTasks()
{
    std::unique_lock<std::recursive_mutex> lock(tasks_mtx_);
    while(tasks_.empty()) {
        work_available_.wait_for(lock, std::chrono::seconds(1));

        if(!running_) {
            return false;
        }
    }

    return true;
}

void ThreadGroup::handlePause()
{
    std::unique_lock<std::recursive_mutex> state_lock(state_mtx_);
    while(running_ && pause_) {
        pause_changed_.wait(state_lock);
    }
}

bool ThreadGroup::executeNextTask()
{
    std::unique_lock<std::recursive_mutex> tasks_lock(tasks_mtx_);
    if(!tasks_.empty()) {
        auto task = tasks_.front();
        tasks_.pop_front();

        tasks_lock.unlock();

        {
            std::unique_lock<std::recursive_mutex> state_lock(state_mtx_);
            if(running_) {
                state_lock.unlock();

                executeTask(task);
                return true;
            }
        }
    }

    return false;
}

void ThreadGroup::executeTask(const TaskPtr& task)
{
    try {
        std::unique_lock<std::recursive_mutex> state_lock(execution_mtx_);
        task->execute();

    } catch(const std::exception& e) {
        TaskGenerator* gen = task->getParent();
        if(gen) {
            gen->setError(e.what());
        }
    } catch(const std::string& s) {
        std::cerr << "Uncatched exception (string) exception: " << s << std::endl;
    } catch(...) {
        std::cerr << "Uncatched exception of unknown type and origin!" << std::endl;
        std::abort();
    }
}

