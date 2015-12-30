#ifndef SCHEDULER_H
#define SCHEDULER_H

/// PROJECT
#include <csapex/scheduling/scheduling_fwd.h>

/// SYSTEM
#include <vector>
#include <csapex/utility/slim_signal.hpp>

namespace csapex
{

class Scheduler
{
public:
    virtual ~Scheduler();

    virtual int id() const = 0;
    virtual std::string name() const = 0;

    virtual void setPause(bool pause) = 0;

    virtual void setSteppingMode(bool stepping) = 0;
    virtual bool isStepping() const = 0;
    virtual bool isStepDone() const = 0;
    virtual void step() = 0;

    virtual void stop() = 0;
    virtual void clear() = 0;

    virtual bool isEmpty() const = 0;

    virtual void add(TaskGenerator* schedulable) = 0;
    virtual void add(TaskGenerator* schedulable, const std::vector<TaskPtr>& initial_tasks) = 0;
    virtual std::vector<TaskPtr> remove(TaskGenerator* schedulable) = 0;

    virtual void schedule(TaskPtr schedulable) = 0;

public:
    csapex::slim_signal::Signal<void()> begin_step;
    csapex::slim_signal::Signal<void()> end_step;
};

}

#endif // SCHEDULER_H
