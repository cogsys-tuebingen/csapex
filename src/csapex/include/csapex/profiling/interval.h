#ifndef INTERVAL_H
#define INTERVAL_H

/// COMPONENT
#include <csapex/csapex_profiling_export.h>

/// SYSTEM
#include <map>
#include <chrono>
#include <memory>
#include <vector>

namespace csapex
{
class Timer;

class CSAPEX_PROFILING_EXPORT Interval {
    friend class Timer;

public:
    typedef std::shared_ptr<Interval> Ptr;

public:
    Interval(const std::string& name);

    long getStartMs() const;
    long getEndMs() const;
    long getStartMicro() const;
    long getEndMicro() const;

    void start();
    void stop();

    std::string name() const;

    double lengthMs() const;
    double lengthSubMs() const;

    void entries(std::vector<std::pair<std::string, double> > &out) const;

    void setActive(bool active);
    bool isActive() const;

public:
    std::map<std::string, Interval::Ptr> sub;

private:
    std::string name_;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;
    std::chrono::time_point<std::chrono::high_resolution_clock> end_;
    long length_micro_seconds_;

    bool active_;
};

}

#endif // INTERVAL_H
