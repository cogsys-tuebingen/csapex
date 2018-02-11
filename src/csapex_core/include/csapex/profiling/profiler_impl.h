#ifndef PROFILER_IMPL_H
#define PROFILER_IMPL_H

/// COMPONENT
#include <csapex/profiling/profiler.h>

namespace csapex
{

class CSAPEX_PROFILING_EXPORT ProfilerImplementation : public Profiler
{
public:
    ProfilerImplementation(bool enabled = true, int history = 1);
};

}

#endif // PROFILER_IMPL_H
