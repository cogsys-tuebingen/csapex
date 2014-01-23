/// HEADER
#include <csapex/utility/timable.h>

using namespace csapex;

Timable::Timable()
    : publish_timer_(NULL)
{

}

void Timable::useTimer(Timer *timer)
{
    publish_timer_ = timer;
}
