#ifndef INTERLUDE_H
#define INTERLUDE_H

/// COMPONENT
#include <csapex/profiling/interval.h>
#include <csapex/csapex_profiling_export.h>

/// SYSTEM
#include <memory>

namespace csapex
{

class Timer;

class CSAPEX_PROFILING_EXPORT Interlude {
public:
    typedef std::shared_ptr<Interlude> Ptr;

public:
    Interlude(Timer* parent, const std::string& name);
    Interlude(const std::shared_ptr<Timer>& parent, const std::string& name);
    ~Interlude();

private:
    Interlude(const Interlude& copy) = delete;
    Interlude& operator = (const Interlude& copy) = delete;

private:
    Timer* parent_;
    Interval::Ptr interval_;
};


}

#endif // INTERLUDE_H
