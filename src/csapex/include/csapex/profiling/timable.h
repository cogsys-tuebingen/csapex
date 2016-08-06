#ifndef TIMABLE_H
#define TIMABLE_H

/// SYSTEM
#include <memory>

namespace csapex
{

class Timer;

class Timable
{
public:
    Timable();
    Timable(std::shared_ptr<Timer> timer);

    virtual void useTimer(std::shared_ptr<Timer> timer);
    std::shared_ptr<Timer> getTimer();

protected:
    std::shared_ptr<Timer> profiling_timer_;
};

}

#endif // TIMABLE_H
