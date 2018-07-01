#ifndef SCHEDULER_H
#define SCHEDULER_H

/// PROJECT
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex_core/csapex_core_export.h>

/// SYSTEM
#include <vector>
#include <csapex/utility/slim_signal.hpp>

namespace csapex
{
class CSAPEX_CORE_EXPORT Scheduler
{
public:
    virtual ~Scheduler();

    virtual int id() const = 0;
    virtual std::string getName() const = 0;
    virtual void setName(const std::string& name) = 0;

    virtual void setPause(bool pause) = 0;

    virtual bool canStartStepping() const = 0;
    virtual void setSteppingMode(bool stepping) = 0;
    virtual bool isStepping() const = 0;
    virtual bool isStepDone() const = 0;
    virtual void step() = 0;

    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void clear() = 0;

    virtual bool isEmpty() const = 0;

    virtual void add(TaskGeneratorPtr schedulable) = 0;
    virtual void add(TaskGeneratorPtr schedulable, const std::vector<TaskPtr>& initial_tasks) = 0;
    virtual std::vector<TaskPtr> remove(TaskGenerator* schedulable) = 0;

    virtual void schedule(TaskPtr schedulable) = 0;
    virtual void scheduleDelayed(TaskPtr schedulable, std::chrono::system_clock::time_point time) = 0;

public:
    slim_signal::Signal<void()> stepping_enabled;
    slim_signal::Signal<void()> begin_step;
    slim_signal::Signal<void()> end_step;

    slim_signal::Signal<void()> scheduler_changed;
};

}  // namespace csapex

#endif  // SCHEDULER_H
