#ifndef RATE_H
#define RATE_H

/// PROJECT
#include <csapex/csapex_util_export.h>

/// SYSTEM
#include <chrono>

namespace csapex
{

class CSAPEX_UTILS_EXPORT Rate
{
public:
    Rate(double frequency, bool immediate);

    double getFrequency() const;
    void setFrequency(double f);

    bool isImmediate() const;
    void setImmediate(bool immediate);

    void tick();

public:
    double frequency_;
    bool immediate_;

    std::chrono::system_clock::time_point last_tick_;
};

}

#endif // RATE_H
