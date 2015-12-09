/// HEADER
#include <csapex/scheduling/executor.h>

using namespace csapex;

Executor::Executor()
    : paused_(false), stepping_(false)
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


void Executor::setSteppingMode(bool stepping)
{
    if(stepping == stepping_) {
        return;
    }

    stepping_ = stepping;

    steppingChanged(stepping);

    if(stepping) {
        setPause(false);
    }
}

void Executor::step()
{
    begin_step();

    performStep();
}

bool Executor::isSteppingMode() const
{
    return stepping_;
}
