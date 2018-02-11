/// HEADER
#include <csapex/profiling/timable.h>

/// COMPONENT
#include <csapex/profiling/timer.h>

using namespace csapex;

Timable::Timable()
    : profiling_timer_(std::make_shared<Timer>("null"))
{

}

Timable::Timable(std::shared_ptr<Timer> timer)
    : profiling_timer_(timer)
{

}

void Timable::useTimer(std::shared_ptr<Timer> timer)
{
    apex_assert_hard(timer);
    profiling_timer_ = timer;
}

std::shared_ptr<Timer> Timable::getTimer()
{
    return profiling_timer_;
}
