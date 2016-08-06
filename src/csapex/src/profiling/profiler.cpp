/// HEADER
#include <csapex/profiling/profiler.h>

using namespace csapex;

Profiler::Profiler(const std::string& name)
    : Profiler(std::make_shared<Timer>(name))
{

}

Profiler::Profiler(Timer::Ptr timer)
    : timer_(timer),

      timer_history_pos_(0),
      count_(0)
{
    //    timer_history_length = settings_.get<int>("timer_history_length", 15);
    timer_history_length = 15;
    timer_history_.resize(timer_history_length);
    apex_assert_hard(timer_history_.size() == timer_history_length);
    apex_assert_hard(timer_history_.capacity() == timer_history_length);


    connections_.emplace_back(timer->finished.connect([this](Timer::Interval::Ptr interval){
        timer_history_[timer_history_pos_] = interval;

        if(++timer_history_pos_ >= (int) timer_history_.size()) {
            timer_history_pos_ = 0;
        }

        std::vector<std::pair<std::string, double> > entries;
        interval->entries(entries);
        for(const auto& it : entries) {
            steps_acc_[it.first](it.second);
        }
        ++count_;
    }));
}

Timer::Ptr Profiler::getTimer() const
{
    return timer_;
}

void Profiler::setEnabled(bool enabled)
{
    timer_->setEnabled(enabled);
}

bool Profiler::isEnabled() const
{
    return timer_->isEnabled();
}

void Profiler::reset()
{
    steps_acc_.clear();
    count_ = 0;
    timer_history_pos_ = 0;
}

std::size_t Profiler::count() const
{
    return count_;
}
std::size_t Profiler::size() const
{
    return timer_history_.size();
}

int Profiler::getCurrentIndex() const
{
    return timer_history_pos_;
}

const std::vector<Timer::Interval::Ptr>& Profiler::getIntervals() const
{
    return timer_history_;
}

Timer::Interval::Ptr Profiler::getInterval(const std::size_t index) const
{
    return timer_history_.at(index);
}

Profiler::Stats Profiler::getStats(const std::string& name) const
{
    Stats res;
    res.mean = boost::accumulators::mean(steps_acc_.at(name));
    res.stddev = std::sqrt(boost::accumulators::variance(steps_acc_.at(name)));
    return res;
}
