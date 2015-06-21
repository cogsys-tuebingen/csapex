#ifndef SCHEDULER_H
#define SCHEDULER_H

/// PROJECT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <vector>

namespace csapex
{

class Scheduler
{
public:
    virtual ~Scheduler();

    virtual void setPause(bool pause) = 0;
    virtual void stop() = 0;
    virtual void clear() = 0;

    virtual bool isEmpty() const = 0;

    virtual void add(TaskGenerator* schedulable) = 0;
    virtual void add(TaskGenerator* schedulable, const std::vector<TaskPtr>& initial_tasks) = 0;
    virtual std::vector<TaskPtr> remove(TaskGenerator* schedulable) = 0;

    virtual void schedule(TaskPtr schedulable) = 0;
};

}

#endif // SCHEDULER_H
