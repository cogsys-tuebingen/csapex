/// HEADER
#include <csapex/utility/timer.h>

using namespace csapex;

Timer::Timer()
{
    timer.start();
}

int Timer::elapsedMs()
{
    return timer.elapsed();
}
