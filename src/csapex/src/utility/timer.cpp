/// HEADER
#include <csapex/utility/timer.h>

using namespace csapex;

Timer::Timer(const std::string& name)
    : intervals(name)
{
    intervals.start = QDateTime::currentDateTime();
    current_sub_interval = &intervals;
}
Timer::~Timer()
{
    finish();
}

void Timer::finish()
{
    intervals.end = QDateTime::currentDateTime();
}

Timer::Interlude::Ptr Timer::step(const std::string &name)
{
    return Timer::Interlude::Ptr(new Timer::Interlude(this, name));
}

Timer::Interlude::Interlude(Timer *parent, const std::string &name)
    : parent_(parent)
{
    // start new interval in timer
    Interval i(name);
    i.parent = parent_->current_sub_interval;
    i.start = QDateTime::currentDateTime();

    parent_->current_sub_interval->sub.push_back(i);
    parent_->current_sub_interval = &(parent_->current_sub_interval->sub.back());
}

Timer::Interlude::~Interlude()
{
    // stop interval
    parent_->current_sub_interval->end = QDateTime::currentDateTime();
    parent_->current_sub_interval = parent_->current_sub_interval->parent;
}

int Timer::Interval::lengthMs() const
{
    return end.toMSecsSinceEpoch() - start.toMSecsSinceEpoch();
}

int Timer::Interval::lengthSubMs() const
{
    int sum = 0;
    for(std::vector<Interval>::const_iterator it = sub.begin(); it != sub.end(); ++it) {
        const Interval& i = *it;
        sum += i.lengthMs();
    }
    return sum;
}
