/// HEADER
#include <csapex/profiling/profiler.h>

using namespace csapex;

Profiler::Profiler()
    : enabled_(false)
{

}

Profile::Profile(const std::string& key)
    : timer(std::make_shared<Timer>(key)),
      timer_history_pos_(0),
      count_(0)
{
    //    timer_history_length = settings_.get<int>("timer_history_length", 15);
    timer_history_length = 15;
    timer_history_.resize(timer_history_length);
    apex_assert_hard(timer_history_.size() == timer_history_length);
    apex_assert_hard(timer_history_.capacity() == timer_history_length);
}

Timer::Ptr Profiler::getTimer(const std::string &key)
{
    const Profile& prof = getProfile(key);
    return prof.timer;
}

const Profile& Profiler::getProfile(const std::string& key)
{
    auto pos = profiles_.find(key);
    if(pos == profiles_.end()) {
        profiles_.emplace(key, key);
        Profile& profile = profiles_.at(key);

        profile.timer->finished.connect([this](Interval::Ptr) { updated(); });
        profile.timer->setEnabled(enabled_);

        connections_.emplace_back(profile.timer->finished.connect([this, &profile](Interval::Ptr interval){
            profile.timer_history_[profile.timer_history_pos_] = interval;

            if(++profile.timer_history_pos_ >= (int) profile.timer_history_.size()) {
                profile.timer_history_pos_ = 0;
            }

            std::vector<std::pair<std::string, double> > entries;
            interval->entries(entries);
            for(const auto& it : entries) {
                profile.steps_acc_[it.first](it.second);
            }
            ++profile.count_;
        }));

        return profile;
    }

    return pos->second;
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

void Profiler::setEnabled(bool enabled)
{
    enabled_ = enabled;
    for(auto& pair : profiles_) {
        Profile& profile = pair.second;
        Timer::Ptr timer = profile.timer;
        timer->setEnabled(enabled_);
    }
}

bool Profiler::isEnabled() const
{
    return enabled_;
}

void Profiler::reset()
{
    for(auto& pair : profiles_) {
        Profile& profile = pair.second;
        profile.reset();
    }
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
