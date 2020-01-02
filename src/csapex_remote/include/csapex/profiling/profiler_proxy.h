#ifndef PROFILER_PROXY_H
#define PROFILER_PROXY_H

/// COMPONENT
#include <csapex/profiling/profiler.h>

/// PROJECT
#include <csapex/io/proxy.h>

namespace csapex
{
class CSAPEX_PROFILING_EXPORT ProfilerProxy : public Profiler
{
public:
    ProfilerProxy(io::ChannelPtr node_channel);

    void setEnabled(bool enabled) override;

    void updateInterval(std::shared_ptr<const Interval>& interval);

private:
    io::ChannelPtr node_channel_;
};

}  // namespace csapex

#endif  // PROFILER_PROXY_H
