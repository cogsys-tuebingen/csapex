#ifndef PROFILER_REMOTE_H
#define PROFILER_REMOTE_H

/// COMPONENT
#include <csapex/profiling/profiler.h>

/// PROJECT
#include <csapex/io/remote.h>

namespace csapex
{

class CSAPEX_PROFILING_EXPORT ProfilerRemote: public Profiler
{
public:
    ProfilerRemote(io::ChannelPtr node_channel);

    virtual void setEnabled(bool enabled);

    void updateInterval(std::shared_ptr<const Interval>& interval);

private:
    io::ChannelPtr node_channel_;
};

}

#endif // PROFILER_REMOTE_H
