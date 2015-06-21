/// HEADER
#include <csapex/scheduling/executor.h>

using namespace csapex;

Executor::Executor()
    : paused_(false)
{

}

Executor::~Executor()
{

}

void Executor::setPause(bool pause)
{
    if(pause == paused_) {
        return;
    }
    paused_ = pause;

    pauseChanged(paused_);

    paused(paused_);
}

bool Executor::isPaused() const
{
    return paused_;
}
