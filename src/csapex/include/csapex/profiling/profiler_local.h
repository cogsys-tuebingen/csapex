#ifndef PROFILER_LOCAL_H
#define PROFILER_LOCAL_H

/// COMPONENT
#include <csapex/profiling/profiler.h>

namespace csapex
{

class CSAPEX_PROFILING_EXPORT ProfilerLocal : public Profiler
{
public:
    ProfilerLocal(bool enabled = true, int history = 1);
};

}

#endif // PROFILER_LOCAL_H
