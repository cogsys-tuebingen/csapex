#ifndef INTERLUDE_H
#define INTERLUDE_H

/// COMPONENT
#include <csapex/profiling/interval.h>

/// SYSTEM
#include <memory>

namespace csapex
{

class Timer;

class Interlude {
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
