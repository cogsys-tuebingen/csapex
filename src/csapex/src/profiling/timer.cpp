/// HEADER
#include <csapex/profiling/timer.h>

/// SYSTEM
#include <assert.h>

using namespace csapex;

Timer::Timer(const std::string& name)
    : timer_name_(name), root(new Interval(name)), enabled_(false), dirty_(false)
{
    restart();
}
Timer::~Timer()
{
}

std::vector<std::pair<std::string, double> > Timer::entries() const
{
    assert(active.empty());

    std::vector<std::pair<std::string, double> > result;
    root->entries(result);
    return result;
}

void Timer::setEnabled(bool enabled)
{
    enabled_ = enabled;
    dirty_  = true;
}

bool Timer::isEnabled() const
{
    return enabled_;
}

void Timer::restart()
{
    root.reset(new Interval(timer_name_));
    active.clear();
    active.push_back(root);
}

void Timer::finish()
{
    while(!active.empty()) {
        active.back()->stop();
        active.pop_back();
    }

    if(dirty_) {
        dirty_ = false;
    } else {
        if(enabled_) {
            finished(root);
        }
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


long Timer::elapsedMs() const
{
    auto now = std::chrono::high_resolution_clock::now();
    auto start = root->start_;
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

