#ifndef EXECUTOR_H
#define EXECUTOR_H

/// COMPONENT
#include <csapex/scheduling/scheduling_fwd.h>

/// SYSTEM
#include <csapex/utility/slim_signal.hpp>

namespace csapex
{
class Executor
{
public:
    Executor();
    virtual ~Executor();

    virtual void add(TaskGenerator*) = 0;
    virtual void remove(TaskGenerator *) = 0;

    bool isPaused() const;
    void setPause(bool pause);

    bool isSteppingMode() const;
    void setSteppingMode(bool stepping);
    void step();

    virtual void stop() = 0;
    virtual void clear() = 0;

protected:
    virtual void pauseChanged(bool pause) = 0;
    virtual void steppingChanged(bool performStep) = 0;
    virtual void performStep() = 0;

public:
    csapex::slim_signal::Signal<void (bool)> paused;

    csapex::slim_signal::Signal<void ()> begin_step;
    csapex::slim_signal::Signal<void ()> end_step;

private:
    bool paused_;
    bool stepping_;
};
}

#endif // EXECUTOR_H

