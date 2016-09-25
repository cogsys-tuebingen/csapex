#ifndef TIMER_H
#define TIMER_H

/// PROJECT
#include <csapex/utility/slim_signal.hpp>
#include <csapex/profiling/interval.h>
#include <csapex/profiling/interlude.h>
#include <csapex/csapex_profiling_export.h>

/// SYSTEM
#include <chrono>
#include <memory>
#include <vector>
#include <deque>
#include <map>

namespace csapex
{

class CSAPEX_PROFILING_EXPORT Timer
{
public:
    typedef std::shared_ptr<Timer> Ptr;

public:
    slim_signal::Signal<void(Interval::Ptr)> finished;

public:
    Timer(const std::string &name, bool enabled = true);
    ~Timer();

    void setEnabled(bool enabled);
    bool isEnabled() const;

    bool isFinished() const;

    void restart();
    void finish();
    std::vector<std::pair<std::string, double> > entries() const;

    Interlude::Ptr step(const std::string& name);

    long startTimeMs() const;
    long stopTimeMs() const;

    long elapsedMs() const;

public:
    std::string timer_name_;

    Interval::Ptr root;
    std::deque<Interval::Ptr> active;

    bool enabled_;
    bool dirty_;
    bool finished_;
};

}

#endif // TIMER_H
