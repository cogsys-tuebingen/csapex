/// HEADER
#include <csapex/profiling/trace.h>

/// COMPONENT
#include <csapex/profiling/timer.h>

using namespace csapex;

Trace::Ptr Timer::step(const std::string& name)
{
    return Trace::Ptr(new Trace(this, name));
}
Trace::Trace(const std::shared_ptr<Timer>& parent, const std::string& name) : Trace(parent.get(), name)
{
}

Trace::Trace(Timer* parent, const std::string& name) : parent_(parent)
{
    // start new interval in timer
    if (parent_->active.empty()) {
        parent_->active.emplace_back(std::make_shared<Interval>(name));
    }

    if (parent_->active.back()->sub.find(name) == parent_->active.back()->sub.end()) {
        interval_ = std::make_shared<Interval>(name);
        parent_->active.back()->sub[name] = interval_;

    } else {
        interval_ = parent_->active.back()->sub[name];
    }
    interval_->start();
    parent_->active.push_back(interval_);
}

Trace::~Trace()
{
    // stop interval
    //    Interval::Ptr i = parent_->active.back();
    if (!interval_->isStopped()) {
        interval_->stop();
        parent_->active.pop_back();
    }
}
