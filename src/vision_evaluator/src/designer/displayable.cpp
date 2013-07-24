/// HEADER
#include "displayable.h"

/// COMPONENT
#include "box.h"

using namespace vision_evaluator;

Displayable::Displayable()
    : error_(false)
{
}

void Displayable::setError(bool e, const std::string& msg)
{
    QString err;
    if(e) {
        unsigned line = 60;
        for(unsigned i = 0; i < msg.size(); ++i) {
            err += msg[i];
            if((i%line) == 0 && i != 0) {
                err += '\n';
            }
        }
    }
    box_->setToolTip(err);
    error_ = e;

    errorEvent(error_);
}

bool Displayable::isError()
{
    return error_;
}


void Displayable::setBox(Box* box)
{
    box_ = box;
}

Box* Displayable::getBox()
{
    return box_;
}

void Displayable::errorEvent(bool error)
{

}
