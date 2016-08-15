#ifndef PROFILABLE_H
#define PROFILABLE_H

/// COMPONENT
#include <csapex/csapex_profiling_export.h>

/// SYSTEM
#include <memory>

namespace csapex
{

class Profiler;

class CSAPEX_PROFILING_EXPORT Profilable
{
public:
    Profilable();
    Profilable(std::shared_ptr<Profiler> profiler);

    virtual void useProfiler(std::shared_ptr<Profiler> profiler);
    std::shared_ptr<Profiler> getProfiler();

protected:
    std::shared_ptr<Profiler> profiler_;
};

}

#endif // PROFILABLE_H
