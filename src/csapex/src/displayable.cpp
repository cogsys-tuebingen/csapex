/// HEADER
#include <csapex/displayable.h>

/// COMPONENT
#include <csapex/box.h>

using namespace csapex;

Displayable::Displayable()
    : error_(false)
{
}

Displayable::~Displayable()
{

}

void Displayable::setError(bool e, const std::string& msg, ErrorLevel level)
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
    level_ = level;

    errorEvent(error_, level_);
}

bool Displayable::isError() const
{
    return error_;
}

Displayable::ErrorLevel Displayable::errorLevel() const
{
    return level_;
}

void Displayable::setBox(Box* box)
{
    box_ = box;
}

Box* Displayable::getBox() const
{
    return box_;
}

void Displayable::errorEvent(bool, ErrorLevel)
{

}
