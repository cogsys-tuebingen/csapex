#ifndef EXECUTOR_H
#define EXECUTOR_H

/// SYSTEM
#include <boost/signals2/signal.hpp>

namespace csapex
{
class Executor
{
public:
    Executor();
    virtual ~Executor();

    bool isPaused() const;
    void setPause(bool pause);

    virtual void clear() = 0;

protected:
    virtual void pauseChanged(bool pause) = 0;

public:
    boost::signals2::signal<void (bool)> paused;

private:
    bool paused_;
};
}

#endif // EXECUTOR_H

