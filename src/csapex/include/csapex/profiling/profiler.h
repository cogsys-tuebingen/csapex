#ifndef PROFILER_H
#define PROFILER_H

/// COMPONENT
#include <csapex/profiling/timer.h>
#include <csapex/profiling/profile.h>
#include <csapex/csapex_profiling_export.h>

/// SYSTEM
#include <map>

namespace csapex
{

class CSAPEX_PROFILING_EXPORT Profiler
{
public:

public:
    slim_signal::Signal<void()> updated;

public:
    Profiler();

    void setEnabled(bool enabled);
    bool isEnabled() const;

    void reset();

    Timer::Ptr getTimer(const std::string& key);
    const Profile& getProfile(const std::string& key);

private:
    std::map<std::string, Profile> profiles_;
    std::vector<slim_signal::ScopedConnection> connections_;

    bool enabled_;
};

}

#endif // PROFILER_H
