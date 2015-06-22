/// HEADER
#include <csapex/utility/timer.h>

/// SYSTEM
#include <assert.h>

using namespace csapex;

Timer::Timer(const std::string& name)
    : timer_name_(name), root(new Interval(name))
{
    active.push_back(root);
}
Timer::~Timer()
{
    finish();
}

std::vector<std::pair<std::string, int> > Timer::entries() const
{
    assert(active.empty());

    std::vector<std::pair<std::string, int> > result;
    root->entries(result);
    return result;
}

void Timer::finish()
{
    while(!active.empty()) {
        active.back()->stop();
        active.pop_back();
    }
}

long Timer::startTimeMs() const
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(root->start_.time_since_epoch()).count();
}

long Timer::stopTimeMs() const
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(root->end_.time_since_epoch()).count();
}

Timer::Interlude::Ptr Timer::step(const std::string &name)
{
    return Timer::Interlude::Ptr(new Timer::Interlude(this, name));
}

Timer::Interlude::Interlude(Timer *parent, const std::string &name)
    : parent_(parent)
{
    // start new interval in timer
    if(parent_->active.back()->sub.find(name) == parent_->active.back()->sub.end()) {
        interval_.reset(new Interval(name));
        parent_->active.back()->sub[name] = interval_;

    } else {
        interval_ = parent_->active.back()->sub[name];

    }
    interval_->start();
    parent_->active.push_back(interval_);
}

Timer::Interlude::~Interlude()
{
    // stop interval
//    Interval::Ptr i = parent_->active.back();
    interval_->stop();
    parent_->active.pop_back();
}

void Timer::Interval::entries(std::vector<std::pair<std::string, int> > &out) const
{
    out.push_back(std::make_pair(name_, lengthMs()));
    for(std::map<std::string, Interval::Ptr>::const_iterator it = sub.begin(); it != sub.end(); ++it) {
        it->second->entries(out);
    }
}

int Timer::Interval::lengthMs() const
{
    return length_;
}

int Timer::Interval::lengthSubMs() const
{
    int sum = 0;
    for(std::map<std::string, Interval::Ptr>::const_iterator it = sub.begin(); it != sub.end(); ++it) {
        const Interval& i = *it->second;
        sum += i.lengthMs();
    }
    return sum;
}

Timer::Interval::Interval(const std::string &name)
    : name_(name), length_(0)
{
    start();
}

std::string Timer::Interval::name() const
{
    return name_;
}

void Timer::Interval::start()
{
    start_ = std::chrono::system_clock::now();
}

void Timer::Interval::stop()
{
    end_ = std::chrono::system_clock::now();
    length_ += std::chrono::duration_cast<std::chrono::milliseconds>(end_ - start_).count();
}
