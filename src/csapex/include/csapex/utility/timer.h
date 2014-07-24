#ifndef TIMER_H
#define TIMER_H

/// SYSTEM
#include <QTime>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <deque>

namespace csapex
{

class Timer
{
public:
    typedef boost::shared_ptr<Timer> Ptr;

    struct Interval {
        typedef boost::shared_ptr<Interval> Ptr;

        Interval(const std::string& name);

        void start();
        void stop();

        std::string name() const;

        int lengthMs() const;
        int lengthSubMs() const;

        void entries(std::vector<std::pair<std::string, int> > &out) const;

    public:
        std::vector<Interval::Ptr> sub;

    private:
        std::string name_;
        QDateTime start_;
        QDateTime end_;
    };

    class Interlude : public boost::noncopyable {
    public:
        typedef boost::shared_ptr<Interlude> Ptr;

    public:
        Interlude(Timer* parent, const std::string& name);
        ~Interlude();
    private:
        Timer* parent_;
    };

public:
    Timer(const std::string &name);
    ~Timer();

    void finish();
    std::vector<std::pair<std::string, int> > entries() const;

    Interlude::Ptr step(const std::string& name);

public:
    std::string timer_name_;

    Interval::Ptr root;
    std::deque<Interval::Ptr> active;
};

}

#endif // TIMER_H
