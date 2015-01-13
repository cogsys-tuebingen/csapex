#ifndef TIMER_H
#define TIMER_H

/// SYSTEM
#include <QTime>
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

        int lengthMs() const;
        int lengthSubMs() const;

        void entries(std::vector<std::pair<std::string, int> > &out) const;

    public:
        std::map<std::string, Interval::Ptr> sub;

    private:
        std::string name_;
        QDateTime start_;
        QDateTime end_;
        int length_;
    };

    class Interlude : public boost::noncopyable {
    public:
        typedef std::shared_ptr<Interlude> Ptr;

    public:
        Interlude(Timer* parent, const std::string& name);
        ~Interlude();
    private:
        Timer* parent_;
        Interval::Ptr interval_;
    };

public:
    Timer(const std::string &name);
    ~Timer();

    void finish();
    std::vector<std::pair<std::string, int> > entries() const;

    Interlude::Ptr step(const std::string& name);

    long startTimeMs() const;
    long stopTimeMs() const;

public:
    std::string timer_name_;

    Interval::Ptr root;
    std::deque<Interval::Ptr> active;
};

}

#endif // TIMER_H
