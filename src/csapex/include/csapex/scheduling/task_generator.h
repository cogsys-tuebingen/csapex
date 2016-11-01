#ifndef TASK_GENERATOR_H
#define TASK_GENERATOR_H

/// PROJECT
#include <csapex/utility/uuid.h>
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/csapex_export.h>

/// SYSTEM
#include <csapex/utility/slim_signal.hpp>

namespace csapex
{

class CSAPEX_EXPORT TaskGenerator : public std::enable_shared_from_this<TaskGenerator>
{
public:
    virtual ~TaskGenerator();

    virtual void assignToScheduler(Scheduler* scheduler) = 0;
    virtual Scheduler* getScheduler() const = 0;
    virtual void detach() = 0;

    virtual bool isPaused() const = 0;
    virtual void setPause(bool pause) = 0;

    virtual void setSteppingMode(bool stepping) = 0;
    virtual void step() = 0;
    virtual bool isStepping() const = 0;
    virtual bool isStepDone() const = 0;

    virtual UUID getUUID() const = 0;

    virtual void setError(const std::string& msg) = 0;

    virtual void reset() = 0;

public:
    csapex::slim_signal::Signal<void()> begin_step;
    csapex::slim_signal::Signal<void()> end_step;
};

}

#endif // TASK_GENERATOR_H
