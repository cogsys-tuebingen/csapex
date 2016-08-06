#ifndef TIMER_H
#define TIMER_H

/// PROJECT
#include <csapex/utility/slim_signal.hpp>

/// SYSTEM
#include <chrono>
#include <boost/noncopyable.hpp>
#include <memory>
#include <vector>
#include <deque>
#include <map>

namespace csapex
{

class Timer
{
public:
    typedef std::shared_ptr<Timer> Ptr;

    struct Interval {
        friend class Timer;

        typedef std::shared_ptr<Interval> Ptr;

        Interval(const std::string& name);

        void start();
        void stop();

        std::string name() const;

        double lengthMs() const;
        double lengthSubMs() const;

        void entries(std::vector<std::pair<std::string, double> > &out) const;

    public:
        std::map<std::string, Interval::Ptr> sub;

    private:
        std::string name_;
        std::chrono::time_point<std::chrono::high_resolution_clock> start_;
        std::chrono::time_point<std::chrono::high_resolution_clock> end_;
        long length_micro_seconds_;
    };

    class Interlude : public boost::noncopyable {
    public:
        typedef std::shared_ptr<Interlude> Ptr;

    public:
        Interlude(Timer* parent, const std::string& name);
        Interlude(const std::shared_ptr<Timer>& parent, const std::string& name);
        ~Interlude();
    private:
        Timer* parent_;
        Interval::Ptr interval_;
    };

public:
    slim_signal::Signal<void(Interval::Ptr)> finished;

public:
    Timer(const std::string &name);
    ~Timer();

    void setEnabled(bool enabled);
    bool isEnabled() const;

    void restart();
    void finish();
    std::vector<std::pair<std::string, double> > entries() const;

    Interlude::Ptr step(const std::string& name);

    long startTimeMs() const;
    long stopTimeMs() const;

public:
    std::string timer_name_;

    Interval::Ptr root;
    std::deque<Interval::Ptr> active;

    bool enabled_;
};

}

#endif // TIMER_H
