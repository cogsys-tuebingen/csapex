/// HEADER
#include <csapex/scheduling/thread_group.h>

/// PROJECT
#include <csapex/scheduling/task.h>
#include <csapex/utility/thread.h>
#include <csapex/scheduling/task_generator.h>
#include <csapex/utility/assert.h>
#include <csapex/utility/cpu_affinity.h>
#include <csapex/utility/exceptions.h>
#include <csapex/core/exception_handler.h>
#include <csapex/scheduling/timed_queue.h>
#include <csapex/profiling/profiler.h>
#include <csapex/profiling/trace.h>
#include <csapex/utility/yaml.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

int ThreadGroup::next_id_ = ThreadGroup::MINIMUM_THREAD_ID;

ThreadGroup::ThreadGroup(TimedQueuePtr timed_queue, ExceptionHandler& handler, int id, std::string name)
  : handler_(handler), destroyed_(false), id_(id), name_(name), cpu_affinity_(new CpuAffinity), timed_queue_(timed_queue), running_(false), pause_(false), stepping_(false)
{
    next_id_ = std::max(next_id_, id + 1);
    setup();
}
ThreadGroup::ThreadGroup(TimedQueuePtr timed_queue, ExceptionHandler& handler, std::string name)
  : handler_(handler), destroyed_(false), id_(next_id_++), name_(name), cpu_affinity_(new CpuAffinity), timed_queue_(timed_queue), running_(false), pause_(false), stepping_(false)
{
    setup();
}

ThreadGroup::~ThreadGroup()
{
    std::vector<TaskGeneratorPtr> generators_copy = generators_;
    for (const TaskGeneratorPtr& tg : generators_copy) {
        tg->detach();
    }
    if (running_ || scheduler_thread_.joinable()) {
        stop();
    }
    destroyed_ = true;
}

void ThreadGroup::setup()
{
    cpu_affinity_->affinity_changed.connect([this](const CpuAffinity*) { updateAffinity(); });
}

void ThreadGroup::updateAffinity()
{
    if (!scheduler_thread_.joinable()) {
        return;
    }

#if WIN32
    // TODO: implement for other platforms
#else
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    const std::vector<bool>& cpus = getCpuAffinity()->get();
    for (std::size_t cpu = 0, n = cpus.size(); cpu < n; ++cpu) {
        if (cpus[cpu]) {
            CPU_SET(cpu, &cpuset);
        }
    }
    int rc = pthread_setaffinity_np(scheduler_thread_.native_handle(), sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
        std::cerr << "failed to set cpu affinity in thread " << name_ << std::endl;
    }
#endif
}

int ThreadGroup::nextId()
{
    return next_id_;
}

int ThreadGroup::id() const
{
    return id_;
}

std::string ThreadGroup::getName() const
{
    return name_;
}
void ThreadGroup::setName(const std::string& name)
{
    if (name != name_) {
        name_ = name;
        scheduler_changed();
    }
}

CpuAffinityPtr ThreadGroup::getCpuAffinity() const
{
    return cpu_affinity_;
}

const std::thread& ThreadGroup::thread() const
{
    return scheduler_thread_;
}

std::size_t ThreadGroup::size() const
{
    return generators_.size();
}

bool ThreadGroup::isEmpty() const
{
    return generators_.empty();
}

void ThreadGroup::setPause(bool pause)
{
    if (pause != pause_) {
        pause_ = pause;

        for (auto generator : generators_) {
            generator->setPause(pause);
        }

        std::unique_lock<std::recursive_mutex> lock(state_mtx_);
        pause_changed_.notify_all();
    }
}

void ThreadGroup::setSteppingMode(bool stepping)
{
    if (stepping != stepping_) {
        stepping_ = stepping;
    }
    for (auto generator : generators_) {
        generator->setSteppingMode(stepping_);
    }
}

bool ThreadGroup::canStartStepping() const
{
    for (auto generator : generators_) {
        if (!generator->canStartStepping()) {
            return false;
        }
    }
    return true;
}

void ThreadGroup::step()
{
    begin_step();

    std::unique_lock<std::recursive_mutex> state_lock(execution_mtx_);
    for (auto generator : generators_) {
        generator->step();
    }
}

bool ThreadGroup::isStepping() const
{
    // this is only consistency checking...
    int stepping = 0;
    int not_stepping = 0;
    for (auto generator : generators_) {
        if (generator->isStepping()) {
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
    // TRACE std::cerr << " TG =========== " << std::endl;
    for (auto generator : generators_) {
        if (!generator->isStepDone()) {
            // TRACE std::cerr << "++++ " << std::endl;
            // TRACE for(const TaskGeneratorConstPtr& g : generators_) {
            // TRACE if(!g->isStepDone()) {
            // TRACE std::cerr << g->getUUID() << " is not done" << std::endl;
            // TRACE }
            // TRACE }
            return false;
        }
    }

    return true;
}

void ThreadGroup::start()
{
    std::unique_lock<std::recursive_mutex> lock(state_mtx_);
    if (scheduler_thread_.joinable()) {
        running_ = false;
        pause_changed_.notify_all();
        lock.unlock();

        scheduler_thread_.join();
    }

    running_ = true;

    scheduler_thread_ = std::thread([this]() {
        csapex::thread::set_name((name_).c_str());
        updateAffinity();

        schedulingLoop();
    });
}

void ThreadGroup::stop()
{
    {
        std::unique_lock<std::recursive_mutex> lock(state_mtx_);
        running_ = false;
        pause_changed_.notify_all();
    }
    {
        std::unique_lock<std::recursive_mutex> state_lock(execution_mtx_);
    }
    {
        std::unique_lock<std::recursive_mutex> lock(tasks_mtx_);
        work_available_.notify_all();
        if (scheduler_thread_.joinable()) {
            lock.unlock();

            scheduler_thread_.join();
        }

        auto gen = generators_;
        for (const TaskGeneratorPtr& tg : gen) {
            tg->detach();
        }

        apex_assert_hard(generators_.empty());

        generator_connections_.clear();

        clear();
    }
}

bool ThreadGroup::isRunning() const
{
    return running_;
}

void ThreadGroup::clear()
{
    {
        std::unique_lock<std::recursive_mutex> lock(tasks_mtx_);
        tasks_.clear();
    }

    std::unique_lock<std::recursive_mutex> state_lock(execution_mtx_);
    for (auto generator : generators_) {
        generator->reset();
    }
}

void ThreadGroup::add(TaskGeneratorPtr generator)
{
    generator->setPause(pause_);
    generator->setSteppingMode(stepping_);

    std::unique_lock<std::recursive_mutex> lock(state_mtx_);
    generators_.push_back(generator);

    auto& cs = generator_connections_[generator.get()];

    cs.push_back(generator->end_step.connect([this]() { checkIfStepIsDone(); }));

    generator_added(generator);
}

void ThreadGroup::add(TaskGeneratorPtr generator, const std::vector<TaskPtr>& initial_tasks)
{
    add(generator);

    std::unique_lock<std::recursive_mutex> lock(tasks_mtx_);
    for (const TaskPtr& t : initial_tasks) {
        schedule(t);
    }

    work_available_.notify_all();
}

void ThreadGroup::checkIfStepIsDone()
{
    // TRACE std::cerr << " TG CHECK =========== " << getName() << std::endl;
    if (isStepDone()) {
        // TRACE std::cerr << " TG END STEP " << getName() << std::endl;
        end_step();
    }
}

std::vector<TaskPtr> ThreadGroup::remove(TaskGenerator* generator)
{
    std::vector<TaskPtr> remaining_tasks;

    std::unique_lock<std::recursive_mutex> lock(tasks_mtx_);

    TaskGeneratorPtr removed;

    for (auto it = tasks_.begin(); it != tasks_.end();) {
        TaskPtr task = *it;
        if (task->getParent() == generator) {
            remaining_tasks.push_back(task);
            it = tasks_.erase(it);
        } else {
            ++it;
        }
    }

    for (auto it = generators_.begin(); it != generators_.end();) {
        if (it->get() == generator) {
            removed = *it;
            it = generators_.erase(it);
        } else {
            ++it;
        }
    }

    work_available_.notify_all();

    generator_connections_[generator].clear();

    generator_removed(removed);

    return remaining_tasks;
}

void ThreadGroup::schedule(TaskPtr task)
{
    apex_assert_hard(!destroyed_);

    std::unique_lock<std::recursive_mutex> tasks_lock(tasks_mtx_);

    if (!tasks_.empty()) {
        //        for(const TaskPtr& t : tasks_) {
        for (auto it = tasks_.begin(); it != tasks_.end(); ++it) {
            const TaskPtr& t = *it;
            if (t.get() == task.get()) {
                return;
            }
        }
    }

    tasks_.insert(task);

    task->setScheduled(true);

    work_available_.notify_all();
}

void ThreadGroup::scheduleDelayed(TaskPtr schedulable, std::chrono::system_clock::time_point time)
{
    timed_queue_->schedule(shared_from_this(), schedulable, time);
}

void ThreadGroup::schedulingLoop()
{
    while (running_) {
        bool keep_executing = waitForTasks();
        while (running_ && keep_executing) {
            handlePause();

            keep_executing = executeNextTask();
        }
    }
}

bool ThreadGroup::waitForTasks()
{
    std::unique_lock<std::recursive_mutex> lock(tasks_mtx_);
    while (tasks_.empty()) {
        work_available_.wait_for(lock, std::chrono::seconds(1));

        if (!running_) {
            return false;
        }
    }

    return true;
}

void ThreadGroup::handlePause()
{
    std::unique_lock<std::recursive_mutex> state_lock(state_mtx_);
    while (running_ && pause_) {
        pause_changed_.wait(state_lock);
    }
}

bool ThreadGroup::executeNextTask()
{
    std::unique_lock<std::recursive_mutex> tasks_lock(tasks_mtx_);
    if (!tasks_.empty()) {
        TaskPtr task = *tasks_.begin();
        tasks_.erase(tasks_.begin());

        task->setScheduled(false);

        tasks_lock.unlock();

        {
            std::unique_lock<std::recursive_mutex> state_lock(state_mtx_);
            if (running_) {
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
        ProfilerPtr profiler = getProfiler();
        Trace::Ptr interlude;
        if (profiler && profiler->isEnabled()) {
            TimerPtr timer = profiler->getTimer(getName());
            interlude.reset(new Trace(timer, task->getName()));
        }

        task->execute();

    } catch (const std::exception& e) {
        TaskGenerator* gen = task->getParent();
        if (gen) {
            gen->setError(e.what());
        }
    } catch (const std::string& s) {
        std::cerr << "Uncaught exception (string) exception: " << s << std::endl;

    } catch (const csapex::Failure& assertion) {
        handler_.handleAssertionFailure(assertion);

    } catch (...) {
        std::cerr << "Uncaught exception of unknown type and origin in execution of task " << task->getName() << "!" << std::endl;
        throw;
    }
}

std::vector<TaskGeneratorPtr>::iterator ThreadGroup::begin()
{
    return generators_.begin();
}

std::vector<TaskGeneratorPtr>::const_iterator ThreadGroup::begin() const
{
    return generators_.begin();
}

std::vector<TaskGeneratorPtr>::iterator ThreadGroup::end()
{
    return generators_.end();
}

std::vector<TaskGeneratorPtr>::const_iterator ThreadGroup::end() const
{
    return generators_.end();
}

void ThreadGroup::saveSettings(YAML::Node& node)
{
    node["affinity"] = cpu_affinity_->get();
}

void ThreadGroup::loadSettings(const YAML::Node& node)
{
    if (node["affinity"].IsDefined()) {
        std::vector<bool> affinity = node["affinity"].as<std::vector<bool>>();
        cpu_affinity_->set(affinity);
    }
}
