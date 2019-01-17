#ifndef TRACE_H
#define TRACE_H

/// COMPONENT
#include <csapex/profiling/interval.h>
#include <csapex_profiling_export.h>

/// SYSTEM
#include <memory>

namespace csapex
{
class Timer;

class CSAPEX_PROFILING_EXPORT Trace
{
public:
    typedef std::unique_ptr<Trace> Ptr;

public:
    Trace(Timer* parent, const std::string& name);
    Trace(const std::shared_ptr<Timer>& parent, const std::string& name);
    ~Trace();

private:
    Trace(const Trace& copy) = delete;
    Trace& operator=(const Trace& copy) = delete;

private:
    Timer* timer_;
    Interval::Ptr interval_;
};

}  // namespace csapex

#endif  // TRACE_H
