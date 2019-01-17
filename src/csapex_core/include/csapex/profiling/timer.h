#ifndef TIMER_H
#define TIMER_H

/// PROJECT
#include <csapex/utility/slim_signal.hpp>
#include <csapex/profiling/interval.h>
#include <csapex/profiling/trace.h>
#include <csapex_profiling_export.h>

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
    Timer(const std::string& name, bool enabled = true);
    ~Timer();

    void setEnabled(bool enabled);
    bool isEnabled() const;

    bool isFinished() const;

    void restart();
    void finish();
    std::vector<std::pair<std::string, double> > entries() const;

    Trace::Ptr step(const std::string& name);

    long startTimeMs() const;
    long stopTimeMs() const;

    long elapsedMs() const;

    Interval::Ptr pushInterval(const std::string& name);
    void popInterval();

    void setActivity(bool active);

    Interval::Ptr getRoot() const;

private:
    std::string timer_name_;

    Interval::Ptr root_;

    std::mutex active_intervals_mutex_;
    std::deque<Interval::Ptr> active_intervals_;

    bool enabled_;
    bool dirty_;
    bool finished_;
};

}  // namespace csapex

#endif  // TIMER_H
