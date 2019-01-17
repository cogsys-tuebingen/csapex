/// HEADER
#include <csapex/profiling/timer.h>

/// SYSTEM
#include <assert.h>

using namespace csapex;

Timer::Timer(const std::string& name, bool enabled) : timer_name_(name), root_(new Interval(name)), enabled_(enabled), dirty_(false), finished_(true)
{
    restart();
}
Timer::~Timer()
{
}

Interval::Ptr Timer::pushInterval(const std::string& name)
{
    Interval::Ptr result;
    {
        std::unique_lock<std::mutex> lock(active_intervals_mutex_);
        if (active_intervals_.empty()) {
            active_intervals_.emplace_back(std::make_shared<Interval>(name));
        }
        auto& siblings = active_intervals_.back()->sub;
        if (siblings.find(name) == siblings.end()) {
            result = std::make_shared<Interval>(name);
            siblings[name] = result;

        } else {
            result = siblings[name];
        }
        active_intervals_.push_back(result);
    }

    result->start();
    return result;
}

void Timer::popInterval()
{
    std::unique_lock<std::mutex> lock(active_intervals_mutex_);
    active_intervals_.pop_back();
}

void Timer::setActivity(bool active)
{
    root_->setActive(active);
}

Interval::Ptr Timer::getRoot() const
{
    return root_;
}

std::vector<std::pair<std::string, double> > Timer::entries() const
{
    std::vector<std::pair<std::string, double> > result;
    root_->entries(result);
    return result;
}

void Timer::setEnabled(bool enabled)
{
    if (enabled_ != enabled) {
        enabled_ = enabled;
        if (!finished_) {
            dirty_ = true;
        }
    }
}

bool Timer::isEnabled() const
{
    return enabled_;
}

bool Timer::isFinished() const
{
    return finished_;
}

void Timer::restart()
{
    if (!finished_) {
        finish();
    }

    root_.reset(new Interval(timer_name_));
    {
        std::unique_lock<std::mutex> lock(active_intervals_mutex_);
        active_intervals_.push_back(root_);
    }

    finished_ = false;
}

void Timer::finish()
{
    finished_ = true;
    {
        std::unique_lock<std::mutex> lock(active_intervals_mutex_);
        while (!active_intervals_.empty()) {
            active_intervals_.back()->stop();
            active_intervals_.pop_back();
        }
    }

    if (dirty_) {
        dirty_ = false;
    } else {
        if (enabled_) {
            finished(root_);
        }
    }
}

long Timer::startTimeMs() const
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(root_->start_.time_since_epoch()).count();
}

long Timer::stopTimeMs() const
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(root_->end_.time_since_epoch()).count();
}

long Timer::elapsedMs() const
{
    auto now = std::chrono::high_resolution_clock::now();
    auto start = root_->start_;
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}
