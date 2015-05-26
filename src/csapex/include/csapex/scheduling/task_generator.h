#ifndef TASK_GENERATOR_H
#define TASK_GENERATOR_H

/// PROJECT
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

namespace csapex
{

class TaskGenerator
{
public:
    virtual ~TaskGenerator();

    virtual void assignToScheduler(Scheduler* scheduler) = 0;
    virtual void detach() = 0;

    virtual UUID getUUID() const = 0;
};

}

#endif // TASK_GENERATOR_H
