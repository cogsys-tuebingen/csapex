#ifndef PROFILER_H
#define PROFILER_H

/// COMPONENT
#include <csapex/profiling/timer.h>

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

    struct Profile
    {
        friend class Profiler;

    public:
        Profile(const std::string &key);

        Timer::Ptr getTimer() const;

        std::size_t count() const;
        std::size_t size() const;
        int getCurrentIndex() const;

        const std::vector<Timer::Interval::Ptr>& getIntervals() const;
        Timer::Interval::Ptr getInterval(const std::size_t index) const;

        Stats getStats(const std::string &name) const;
        void reset();

    private:
        Timer::Ptr timer;

        std::size_t timer_history_length;
        int timer_history_pos_;

        typedef boost::accumulators::stats<boost::accumulators::tag::variance> stats;
        typedef boost::accumulators::accumulator_set<double, stats > accumulator;
        std::map<std::string, accumulator> steps_acc_;
        std::vector<Timer::Interval::Ptr> timer_history_;
        unsigned int count_;
    };

public:
    slim_signal::Signal<void()> updated;

public:
    Profiler();

    void setEnabled(bool enabled);
    bool isEnabled() const;

    void reset();

    Timer::Ptr getTimer(const std::string& key);
    const Profile& getProfile(const std::string& key);

private:
    std::map<std::string, Profile> profiles_;
    std::vector<slim_signal::ScopedConnection> connections_;

    bool enabled_;
};

}

#endif // PROFILER_H
