/// HEADER
#include <csapex/profiling/interlude.h>

/// COMPONENT
#include <csapex/profiling/timer.h>

using namespace csapex;


Interlude::Ptr Timer::step(const std::string &name)
{
    return Interlude::Ptr(new Interlude(this, name));
}
Interlude::Interlude(const std::shared_ptr<Timer> &parent, const std::string &name)
    : Interlude(parent.get(), name)
{
}

Interlude::Interlude(Timer* parent, const std::string &name)
    : parent_(parent)
{
    // start new interval in timer
    if(parent_->active.empty()) {
        throw std::runtime_error("no active timer");
    }

    if(parent_->active.back()->sub.find(name) == parent_->active.back()->sub.end()) {
        interval_ = std::make_shared<Interval>(name);
        parent_->active.back()->sub[name] = interval_;

    } else {
        interval_ = parent_->active.back()->sub[name];

    }
    interval_->start();
    parent_->active.push_back(interval_);
}

Interlude::~Interlude()
{
    // stop interval
//    Interval::Ptr i = parent_->active.back();
    interval_->stop();
    parent_->active.pop_back();
}
