#ifndef PROFILER_H
#define PROFILER_H

/// COMPONENT
#include <csapex/utility/timer.h>

/// SYSTEM
#define BOOST_PARAMETER_MAX_ARITY 7
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <map>

namespace csapex
{

class Profiler
{
public:
    struct Stats
    {
        double mean;
        double stddev;
    };

public:
    Profiler(const std::string &name);
    Profiler(Timer::Ptr timer);

    void setEnabled(bool enabled);
    bool isEnabled() const;


    void reset();

    Timer::Ptr getTimer() const;

    std::size_t count() const;
    std::size_t size() const;
    int getCurrentIndex() const;

    const std::vector<Timer::Interval::Ptr>& getIntervals() const;
    Timer::Interval::Ptr getInterval(const std::size_t index) const;

    Stats getStats(const std::string &name) const;

private:
    Timer::Ptr timer_;
    std::vector<slim_signal::ScopedConnection> connections_;

    std::size_t timer_history_length;
    int timer_history_pos_;

    typedef boost::accumulators::stats<boost::accumulators::tag::variance> stats;
    typedef boost::accumulators::accumulator_set<double, stats > accumulator;
    std::map<std::string, accumulator> steps_acc_;
    std::vector<Timer::Interval::Ptr> timer_history_;
    unsigned int count_;
};

}

#endif // PROFILER_H
