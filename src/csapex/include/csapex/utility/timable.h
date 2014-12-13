#ifndef TIMABLE_H
#define TIMABLE_H

/// PROJECT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <sstream>

namespace csapex
{

namespace __interlude
{
static std::stringstream& ss()
{
    static std::stringstream s;
    s.str(std::string());
    return s;
}
}

#define INTERLUDE(stream) \
    csapex::Timer::Interlude::Ptr __interlude__##__LINE__ = profiling_timer_ ? profiling_timer_->step(dynamic_cast<std::stringstream &> ((__interlude::ss() << stream)).str()) : csapex::Timer::Interlude::Ptr(static_cast<csapex::Timer::Interlude*>(NULL))

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
