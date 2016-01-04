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

void Executor::addChild(Executor *e)
{
    children_.push_back(e);

    e->setPause(isPaused());
    e->setSteppingMode(isSteppingMode());

    e->end_step.connect(delegate::Delegate0<>(this, &Executor::checkIfStepIsDone));
}

void Executor::setPause(bool pause)
{
    if(pause == paused_) {
        return;
    }
    paused_ = pause;

    for(Executor* child : children_) {
        child->setPause(pause);
    }

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

    for(Executor* child : children_) {
        child->setSteppingMode(stepping);
    }

    steppingChanged(stepping);

    if(stepping) {
        setPause(false);
    }
}

void Executor::step()
{
    begin_step();

    performStep();

    for(Executor* child : children_) {
        child->step();
    }

    checkIfStepIsDone();
}

void Executor::checkIfStepIsDone()
{
    if(!isStepDone()) {
        return;
    }

    for(Executor* child : children_) {
        if(!child->isStepDone()) {
            return;
        }
    }

    end_step();
}

bool Executor::isSteppingMode() const
{
    return stepping_;
}
