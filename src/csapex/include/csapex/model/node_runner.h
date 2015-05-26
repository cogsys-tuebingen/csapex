#ifndef NODE_RUNNER_H
#define NODE_RUNNER_H

/// PROJECT
#include <csapex/scheduling/task_generator.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <vector>
#include <mutex>
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

    virtual UUID getUUID() const override;

    void schedule(TaskPtr task);

private:
    NodeWorkerPtr worker_;
    Scheduler* scheduler_;

    std::mutex mutex_;

    TaskPtr tick_;
    TaskPtr process_;
    TaskPtr prepare_;
    TaskPtr check_;

    std::vector<boost::signals2::connection> connections_;

    std::vector<TaskPtr> remaining_tasks_;
};

}

#endif // NODE_RUNNER_H
