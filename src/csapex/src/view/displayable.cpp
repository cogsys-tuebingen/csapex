/// HEADER
#include <csapex/view/displayable.h>

/// COMPONENT
#include <csapex/model/box.h>

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
    if(!isError() && !e) {
        return;
    }

    setErrorSilent(e, msg, level);

    errorEvent(error_, level_);
}

void Displayable::setErrorSilent(bool e, const std::string &msg, ErrorLevel level)
{
    QMutexLocker lock(&mutex);

    if(!error_ && !e) {
        return;
    }

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
    error_msg_ = msg;
}

bool Displayable::isError() const
{
    QMutexLocker lock(&mutex);
    return error_;
}

Displayable::ErrorLevel Displayable::errorLevel() const
{
    QMutexLocker lock(&mutex);
    return level_;
}

std::string Displayable::errorMessage() const
{
    QMutexLocker lock(&mutex);
    return error_msg_;
}

void Displayable::setBox(Box* box)
{
    QMutexLocker lock(&mutex);
    box_ = box;
}

Box* Displayable::getBox() const
{
    QMutexLocker lock(&mutex);
    return box_;
}

void Displayable::errorEvent(bool, ErrorLevel)
{

}
