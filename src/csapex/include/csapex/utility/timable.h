#ifndef TIMABLE_H
#define TIMABLE_H

namespace csapex
{

class Timer;

class Timable
{
public:
    Timable();
    Timable(Timer* timer);

    virtual void useTimer(Timer* timer);
    Timer* getTimer();

protected:
    Timer* profiling_timer_;
};

}

#endif // TIMABLE_H
