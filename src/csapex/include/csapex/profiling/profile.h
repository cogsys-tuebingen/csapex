#ifndef PROFILE_H
#define PROFILE_H

/// COMPONENT
#include <csapex/profiling/timer.h>
#include <csapex/csapex_profiling_export.h>

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

struct CSAPEX_PROFILING_EXPORT Profile
{
    friend class Profiler;

public:
    Profile(const std::string &key, int timer_history_length = 1, bool enabled = true);

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
