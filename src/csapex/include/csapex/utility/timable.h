#ifndef TIMABLE_H
#define TIMABLE_H

/// PROJECT
#include <csapex/csapex_fwd.h>

namespace csapex
{

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
