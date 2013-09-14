#ifndef TIMER_H
#define TIMER_H

/// SYSTEM
#include <QTime>

namespace csapex
{

class Timer
{
public:
    Timer();
    int elapsedMs();

private:
    QTime timer;
};

}

#endif // TIMER_H
