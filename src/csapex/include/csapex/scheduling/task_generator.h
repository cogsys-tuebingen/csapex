#ifndef TASK_GENERATOR_H
#define TASK_GENERATOR_H

/// PROJECT
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <boost/signals2/signal.hpp>

namespace csapex
{

class TaskGenerator
{
public:
    virtual ~TaskGenerator();

    virtual void assignToScheduler(Scheduler* scheduler) = 0;
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
    boost::signals2::signal<void()> begin_step;
    boost::signals2::signal<void()> end_step;
};

}

#endif // TASK_GENERATOR_H
