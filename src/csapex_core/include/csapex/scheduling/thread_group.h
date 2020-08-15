#ifndef THREAD_GROUP_H
#define THREAD_GROUP_H

/// PROJECT
#include <csapex/scheduling/scheduler.h>
#include <csapex/scheduling/task.h>
#include <csapex/core/core_fwd.h>
#include <csapex/utility/utility_fwd.h>
#include <csapex/profiling/profiling_fwd.h>
#include <csapex/profiling/profilable.h>

/// SYSTEM
#include <string>
#include <thread>
#include <vector>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <set>

namespace YAML
{
class Node;
}

namespace csapex
{
class CSAPEX_CORE_EXPORT ThreadGroup : public Scheduler, public Profilable, public std::enable_shared_from_this<ThreadGroup>
{
public:
    enum
    {
        UNDEFINED_THREAD = -1,
        PRIVATE_THREAD = 0,
        DEFAULT_GROUP_ID = 1,
        MINIMUM_THREAD_ID = 2
    };

public:
    static int nextId();

public:
    ThreadGroup(TimedQueuePtr timed_queue, ExceptionHandler& handler, int id, std::string name);
    ThreadGroup(TimedQueuePtr timed_queue, ExceptionHandler& handler, std::string name);
    ~ThreadGroup() override;

    int id() const override;

    std::string getName() const override;
    void setName(const std::string& name) override;

    CpuAffinityPtr getCpuAffinity() const;

    const std::thread& thread() const;

    std::size_t size() const;
    bool isEmpty() const override;

    void setPause(bool pause) override;
    void setSteppingMode(bool stepping) override;

    bool canStartStepping() const override;
    void step() override;
    bool isStepping() const override;
    bool isStepDone() const override;

    void start() override;
    void stop() override;
    void clear() override;

    bool isRunning() const;

    void add(TaskGeneratorPtr generator) override;
    void add(TaskGeneratorPtr generator, const std::vector<TaskPtr>& initial_tasks) override;

    std::vector<TaskPtr> remove(TaskGenerator* generator) override;

    void schedule(TaskPtr schedulable) override;
    void scheduleDelayed(TaskPtr schedulable, std::chrono::system_clock::time_point time) override;

    std::vector<TaskGeneratorPtr>::iterator begin();
    std::vector<TaskGeneratorPtr>::const_iterator begin() const;
    std::vector<TaskGeneratorPtr>::iterator end();
    std::vector<TaskGeneratorPtr>::const_iterator end() const;

    void saveSettings(YAML::Node&);
    void loadSettings(const YAML::Node&);

public:
    slim_signal::Signal<void(TaskGeneratorPtr)> generator_added;
    slim_signal::Signal<void(TaskGeneratorPtr)> generator_removed;

private:
    void setup();
    void schedulingLoop();
    void updateAffinity();

    bool waitForTasks();
    void handlePause();
    bool executeNextTask();

    void executeTask(const TaskPtr& task);

    void checkIfStepIsDone();

private:
    ExceptionHandler& handler_;

    bool destroyed_;

    static int next_id_;

    int id_;
    std::string name_;

    CpuAffinityPtr cpu_affinity_;

    TimedQueuePtr timed_queue_;

    std::thread scheduler_thread_;

    std::vector<TaskGeneratorPtr> generators_;
    std::map<TaskGenerator*, std::vector<slim_signal::ScopedConnection>> generator_connections_;

    std::condition_variable_any work_available_;
    std::condition_variable_any pause_changed_;

    std::recursive_mutex tasks_mtx_;
    struct greater
    {
        bool operator()(const TaskPtr& a, const TaskPtr& b) const
        {
            return a->getPriority() > b->getPriority();
        }
    };

    std::multiset<TaskPtr, greater> tasks_;

    std::recursive_mutex state_mtx_;
    std::atomic<bool> running_;
    std::atomic<bool> pause_;
    std::atomic<bool> stepping_;

    mutable std::recursive_mutex execution_mtx_;
};

}  // namespace csapex

#endif  // THREAD_GROUP_H
