#ifndef TIMER_H
#define TIMER_H

/// SYSTEM
#include <QTime>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>

namespace csapex
{

class Timer
{
public:
    typedef boost::shared_ptr<Timer> Ptr;

    struct Interval {
        std::string name;
        QDateTime start;
        QDateTime end;

        Interval* parent;
        std::vector<Interval> sub;

        Interval(const std::string& name)
            : name(name), parent(NULL)
        {}

        int lengthMs() const;
        int lengthSubMs() const;
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

    Interlude::Ptr step(const std::string& name);

public:
    Interval intervals;
    Interval* current_sub_interval;
};

}

#endif // TIMER_H
