#ifndef NODE_RUNNER_H
#define NODE_RUNNER_H

/// PROJECT
#include <csapex/scheduling/task_generator.h>
#include <csapex/model/model_fwd.h>
#include <csapex/model/notifier.h>
#include <csapex_core/csapex_core_export.h>
#include <csapex/model/observer.h>

/// SYSTEM
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>

namespace csapex
{
class CSAPEX_CORE_EXPORT NodeRunner : public TaskGenerator, public Observer, public Notifier
{
public:
    NodeRunner(NodeWorkerPtr worker);
    ~NodeRunner();

    void assignToScheduler(Scheduler* scheduler) override;
    Scheduler* getScheduler() const override;
    void detach() override;

    bool isPaused() const override;
    void setPause(bool pause) override;

    bool canStartStepping() const override;
    void setSteppingMode(bool stepping) override;
    void step() override;
    bool isStepping() const override;
    bool isStepDone() const override;

    UUID getUUID() const override;

    void setError(const std::string& msg) override;

    void reset() override;

    void schedule(TaskPtr task);
    void scheduleDelayed(TaskPtr task, std::chrono::system_clock::time_point time);

    void setSuppressExceptions(bool suppress_exceptions) override;

    void setNodeWorker(NodeWorkerPtr worker);

private:
    void connectNodeWorker();

    void measureFrequency();
    void scheduleProcess();
    void checkParameters();
    void execute();
    void notify();

private:
    NodeWorkerPtr worker_;
    NodeHandlePtr nh_;
    Scheduler* scheduler_;

    mutable std::recursive_mutex mutex_;

    bool paused_;
    bool stepping_;
    int possible_steps_;
    bool step_done_;

    TaskPtr check_parameters_;
    TaskPtr execute_;
    TaskPtr notify_processed_;

    std::vector<TaskPtr> remaining_tasks_;

    long guard_;
    double max_frequency_;

    bool waiting_for_execution_;

    bool waiting_for_step_;
    slim_signal::ScopedConnection wait_for_step_connection_;

    bool suppress_exceptions_;
};

}  // namespace csapex

#endif  // NODE_RUNNER_H
