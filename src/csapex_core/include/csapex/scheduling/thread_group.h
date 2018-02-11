#ifndef THREAD_GROUP_H
#define THREAD_GROUP_H

/// PROJECT
#include <csapex/scheduling/scheduler.h>
#include <csapex/scheduling/task.h>
#include <csapex/core/core_fwd.h>
#include <csapex/utility/utility_fwd.h>

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

class CSAPEX_CORE_EXPORT ThreadGroup : public Scheduler, public std::enable_shared_from_this<ThreadGroup>
{
public:
    enum {
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
    ~ThreadGroup();

    int id() const override;

    std::string getName() const override;
    void setName(const std::string& name) override;

    CpuAffinityPtr getCpuAffinity() const;

    const std::thread &thread() const;

    std::size_t size() const;
    virtual bool isEmpty() const override;

    virtual void setPause(bool pause) override;
    virtual void setSteppingMode(bool stepping) override;

    virtual bool canStartStepping() const override;
    virtual void step() override;
    virtual bool isStepping() const override;
    virtual bool isStepDone() const override;

    virtual void start() override;
    virtual void stop() override;
    virtual void clear() override;

    bool isRunning() const;

    virtual void add(TaskGeneratorPtr generator) override;
    virtual void add(TaskGeneratorPtr generator, const std::vector<TaskPtr>& initial_tasks) override;

    virtual std::vector<TaskPtr> remove(TaskGenerator* generator) override;

    virtual void schedule(TaskPtr schedulable) override;    
    virtual void scheduleDelayed(TaskPtr schedulable, std::chrono::system_clock::time_point time) override;

    std::vector<TaskGeneratorPtr>::iterator begin();
    std::vector<TaskGeneratorPtr>::const_iterator begin() const;
    std::vector<TaskGeneratorPtr>::iterator end();
    std::vector<TaskGeneratorPtr>::const_iterator end() const;

    void saveSettings(YAML::Node&);
    void loadSettings(const YAML::Node &);

public:
    slim_signal::Signal<void (TaskGeneratorPtr)> generator_added;
    slim_signal::Signal<void (TaskGeneratorPtr)> generator_removed;

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
        bool operator () (const TaskPtr& a, const TaskPtr& b) {
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

}

#endif // THREAD_GROUP_H

