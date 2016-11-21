/// HEADER
#include <csapex/profiling/profiler.h>

using namespace csapex;

Profiler::Profiler(bool enabled, int history)
    : enabled_(false), history_length_(history)
{
    setEnabled(enabled);
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
        profiles_.emplace(key, Profile(key, history_length_, enabled_));
        Profile& profile = profiles_.at(key);

        profile.timer->finished.connect([this](Interval::Ptr) { updated(); });

        observe(profile.timer->finished, [this, &profile](Interval::Ptr interval){
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
        });

        return profile;
    }

    return pos->second;
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
