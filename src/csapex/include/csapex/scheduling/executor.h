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

    bool isSteppingMode() const;
    void setSteppingMode(bool stepping);
    void step();

    virtual void clear() = 0;

protected:
    virtual void pauseChanged(bool pause) = 0;
    virtual void steppingChanged(bool performStep) = 0;
    virtual void performStep() = 0;

public:
    boost::signals2::signal<void (bool)> paused;

    boost::signals2::signal<void ()> begin_step;
    boost::signals2::signal<void ()> end_step;

private:
    bool paused_;
    bool stepping_;
};
}

#endif // EXECUTOR_H

