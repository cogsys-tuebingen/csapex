#ifndef TIMABLE_H
#define TIMABLE_H

/// PROJECT
#include <csapex/csapex_fwd.h>

namespace csapex
{

#define INTERLUDE(name) \
    csapex::Timer::Interlude::Ptr __interlude__ = publish_timer_->step(name)

class Timable
{
public:
    Timable();
    virtual void useTimer(Timer* timer);

protected:
    Timer* publish_timer_;
};

}

#endif // TIMABLE_H
