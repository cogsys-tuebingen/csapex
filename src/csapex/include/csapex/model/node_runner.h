#ifndef NODE_RUNNER_H
#define NODE_RUNNER_H

/// PROJECT
#include <csapex/scheduling/task_generator.h>
#include <csapex/model/model_fwd.h>
#include <csapex/csapex_export.h>
#include <csapex/model/observer.h>

/// SYSTEM
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>

namespace csapex
{

class CSAPEX_EXPORT NodeRunner : public TaskGenerator, public Observer
{
public:
    NodeRunner(NodeWorkerPtr worker);
    ~NodeRunner();

    virtual void assignToScheduler(Scheduler* scheduler) override;
    virtual Scheduler* getScheduler() const override;
    virtual void detach() override;

    virtual bool isPaused() const override;
    virtual void setPause(bool pause) override;

    virtual bool canStartStepping() const override;
    virtual void setSteppingMode(bool stepping) override;
    virtual void step() override;
    virtual bool isStepping() const override;
    virtual bool isStepDone() const override;

    virtual UUID getUUID() const override;

    virtual void setError(const std::string& msg) override;

    virtual void reset() override;

    void schedule(TaskPtr task);
    void scheduleDelayed(TaskPtr task, std::chrono::system_clock::time_point time);

private:
    void measureFrequency();

    void scheduleProcess();

    void execute();

private:
    NodeWorkerPtr worker_;
    Scheduler* scheduler_;

    mutable std::recursive_mutex mutex_;

    bool paused_;
    bool stepping_;
    int can_step_;
    bool step_done_;

    TaskPtr check_parameters_;
    TaskPtr execute_;

    std::vector<TaskPtr> remaining_tasks_;

    long guard_;
    double max_frequency_;

    bool waiting_for_execution_;

    bool waiting_for_step_;
    slim_signal::ScopedConnection wait_for_step_connection_;
};

}

#endif // NODE_RUNNER_H
