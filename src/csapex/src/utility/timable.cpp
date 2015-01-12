/// HEADER
#include <csapex/utility/timable.h>

using namespace csapex;

Timable::Timable()
    : profiling_timer_(nullptr)
{

}

Timable::Timable(Timer* timer)
    : profiling_timer_(timer)
{

}

void Timable::useTimer(Timer *timer)
{
    profiling_timer_ = timer;
}
