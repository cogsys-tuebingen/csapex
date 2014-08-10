#ifndef TIMABLE_H
#define TIMABLE_H

/// PROJECT
#include <csapex/csapex_fwd.h>
#include <csapex/utility/timer.h>

namespace csapex
{

#define INTERLUDE(name) \
    csapex::Timer::Interlude::Ptr __interlude__##__LINE__ = profiling_timer_ ? profiling_timer_->step(name) : csapex::Timer::Interlude::Ptr(static_cast<csapex::Timer::Interlude*>(NULL))

class Timable
{
public:
    Timable();
    Timable(Timer* timer);
    virtual void useTimer(Timer* timer);

protected:
    Timer* profiling_timer_;
};

}

#endif // TIMABLE_H
