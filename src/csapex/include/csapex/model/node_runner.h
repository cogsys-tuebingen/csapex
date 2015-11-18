#ifndef NODE_RUNNER_H
#define NODE_RUNNER_H

/// PROJECT
#include <csapex/scheduling/task_generator.h>
#include <csapex/model/model_fwd.h>

/// SYSTEM
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <boost/signals2/connection.hpp>

namespace csapex
{

class NodeRunner : public TaskGenerator
{
public:
    NodeRunner(NodeWorkerPtr worker);
    ~NodeRunner();

    virtual void assignToScheduler(Scheduler* scheduler) override;
    virtual void detach() override;

    virtual bool isPaused() const;
    virtual void setPause(bool pause);

    virtual void setSteppingMode(bool stepping) override;
    virtual void step() override;
    virtual bool isStepping() const override;
    virtual bool isStepDone() const override;

    virtual UUID getUUID() const override;

    virtual void setError(const std::string& msg) override;

    virtual void reset() override;

    void schedule(TaskPtr task);

private:
    void tickLoop();

private:
    NodeWorkerPtr worker_;
    Scheduler* scheduler_;

    mutable std::recursive_mutex mutex_;

    bool paused_;
    bool ticking_;
    bool is_source_;
    bool stepping_;
    bool can_step_;

    TaskPtr tick_;
    TaskPtr check_parameters_;
    TaskPtr check_transitions_;

    std::thread ticking_thread_;

    std::vector<boost::signals2::connection> connections_;

    std::atomic<bool> tick_thread_running_;
    std::atomic<bool> tick_thread_stop_;
    std::vector<TaskPtr> remaining_tasks_;
};

}

#endif // NODE_RUNNER_H
