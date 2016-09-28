/// HEADER
#include <csapex/profiling/profile.h>

using namespace csapex;


Profile::Profile(const std::string& key, int timer_history_length, bool enabled)
    : timer(std::make_shared<Timer>(key, enabled)),
      timer_history_pos_(0),
      count_(0)
{
    timer_history_.resize(timer_history_length);
    apex_assert_hard((int) timer_history_.size() == timer_history_length);
    apex_assert_hard((int) timer_history_.capacity() == timer_history_length);
}

void Profile::reset()
{
    for(auto& pair : steps_acc_) {
        accumulator& acc = pair.second;
        acc = accumulator();
    }
    count_ = 0;
    timer_history_pos_ = 0;
}


Timer::Ptr Profile::getTimer() const
{
    return timer;
}

std::size_t Profile::count() const
{
    return count_;
}
std::size_t Profile::size() const
{
    return timer_history_.size();
}

int Profile::getCurrentIndex() const
{
    return timer_history_pos_;
}

const std::vector<Interval::Ptr>& Profile::getIntervals() const
{
    return timer_history_;
}

Interval::Ptr Profile::getInterval(const std::size_t index) const
{
    return timer_history_.at(index);
}

ProfilerStats Profile::getStats(const std::string& name) const
{
    ProfilerStats res;
    res.mean = boost::accumulators::mean(steps_acc_.at(name));
    res.stddev = std::sqrt(boost::accumulators::variance(steps_acc_.at(name)));
    return res;
}
