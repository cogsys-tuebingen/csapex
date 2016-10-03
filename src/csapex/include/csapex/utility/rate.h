#ifndef RATE_H
#define RATE_H

/// PROJECT
#include <csapex/csapex_util_export.h>

/// SYSTEM
#include <chrono>
#include <deque>

namespace csapex
{

class CSAPEX_UTILS_EXPORT Rate
{
public:
    Rate(double frequency, bool immediate);
    Rate();

    void tick();

    double getEffectiveFrequency() const;
    double getFrequency() const;
    void setFrequency(double f);

    bool isImmediate() const;
    void setImmediate(bool immediate);

    void keepUp();


public:
    double frequency_;
    bool immediate_;

    std::chrono::system_clock::time_point last_scheduled_tick_;
    std::deque<std::chrono::system_clock::time_point> real_ticks_;
};

}

#endif // RATE_H
