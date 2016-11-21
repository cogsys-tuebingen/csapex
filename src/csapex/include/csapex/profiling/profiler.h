#ifndef PROFILER_H
#define PROFILER_H

/// COMPONENT
#include <csapex/profiling/timer.h>
#include <csapex/profiling/profile.h>
#include <csapex/csapex_profiling_export.h>
#include <csapex/model/observer.h>

/// SYSTEM
#include <map>

namespace csapex
{

class CSAPEX_PROFILING_EXPORT Profiler : public Observer
{
public:

public:
    slim_signal::Signal<void()> updated;

public:
    Profiler(bool enabled = true, int history = 1);

    void setEnabled(bool enabled);
    bool isEnabled() const;

    void reset();

    Timer::Ptr getTimer(const std::string& key);
    const Profile& getProfile(const std::string& key);

private:
    std::map<std::string, Profile> profiles_;

    bool enabled_;
    std::size_t history_length_;
};

}

#endif // PROFILER_H
