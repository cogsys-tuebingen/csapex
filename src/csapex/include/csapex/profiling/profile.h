#ifndef PROFILE_H
#define PROFILE_H

/// COMPONENT
#include <csapex/profiling/timer.h>

/// SYSTEM
#define BOOST_PARAMETER_MAX_ARITY 7
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

namespace csapex
{

struct ProfilerStats
{
    double mean;
    double stddev;
};

struct Profile
{
    friend class Profiler;

public:
    Profile(const std::string &key);

    Timer::Ptr getTimer() const;

    std::size_t count() const;
    std::size_t size() const;
    int getCurrentIndex() const;

    const std::vector<Interval::Ptr>& getIntervals() const;
    Interval::Ptr getInterval(const std::size_t index) const;

    ProfilerStats getStats(const std::string &name) const;
    void reset();

private:
    Timer::Ptr timer;

    std::size_t timer_history_length;
    int timer_history_pos_;

    typedef boost::accumulators::stats<boost::accumulators::tag::variance> stats;
    typedef boost::accumulators::accumulator_set<double, stats > accumulator;
    std::map<std::string, accumulator> steps_acc_;
    std::vector<Interval::Ptr> timer_history_;
    unsigned int count_;
};

}

#endif // PROFILE_H
