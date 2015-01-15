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
#define CONCATENATE_DETAIL(x, y) x##y
#define CONCATENATE(x, y) CONCATENATE_DETAIL(x, y)
#define MAKE_UNIQUE(x) CONCATENATE(x, __COUNTER__)

#define INTERLUDE(stream) \
    csapex::Timer::Interlude::Ptr MAKE_UNIQUE(__interlude__) = profiling_timer_ ? profiling_timer_->step(dynamic_cast<std::stringstream &> ((__interlude::ss() << stream)).str()) : csapex::Timer::Interlude::Ptr(static_cast<csapex::Timer::Interlude*>(nullptr))

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
