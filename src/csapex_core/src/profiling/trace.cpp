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

Trace::Trace(Timer* parent, const std::string& name) : timer_(parent)
{
    // start new interval in timer
    interval_ = timer_->pushInterval(name);
}

Trace::~Trace()
{
    if (!interval_->isStopped()) {
        interval_->stop();
        timer_->popInterval();
    }
}
