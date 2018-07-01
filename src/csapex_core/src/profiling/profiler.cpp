/// HEADER
#include <csapex/profiling/profiler.h>

using namespace csapex;

Profiler::Profiler(bool enabled, int history) : enabled_(false), history_length_(history)
{
    apex_assert_hard(history > 0);
    setEnabled(enabled);
}

Timer::Ptr Profiler::getTimer(const std::string& key)
{
    const Profile& prof = getProfile(key);
    return prof.timer;
}

const Profile& Profiler::getProfile(const std::string& key)
{
    auto pos = profiles_.find(key);
    if (pos == profiles_.end()) {
        profiles_.emplace(key, Profile(key, history_length_, enabled_));
        Profile& profile = profiles_.at(key);

        profile.timer->finished.connect([this](Interval::Ptr) { updated(); });

        observe(profile.timer->finished, [this, &profile](Interval::Ptr interval) { profile.addInterval(interval); });

        return profile;
    }

    return pos->second;
}

void Profiler::setEnabled(bool enabled)
{
    if (enabled == enabled_) {
        return;
    }

    enabled_ = enabled;
    for (auto& pair : profiles_) {
        Profile& profile = pair.second;
        Timer::Ptr timer = profile.timer;
        timer->setEnabled(enabled_);
    }

    enabled_changed(enabled_);
}

bool Profiler::isEnabled() const
{
    return enabled_;
}

void Profiler::reset()
{
    for (auto& pair : profiles_) {
        Profile& profile = pair.second;
        profile.reset();
    }
}
