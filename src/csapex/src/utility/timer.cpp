/// HEADER
#include <csapex/utility/timer.h>

using namespace csapex;

Timer::Timer()
{
}

int Timer::elapsedMs()
{
    return timer.elapsed();
}
