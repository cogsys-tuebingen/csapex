/// HEADER
#include <csapex/model/error_state.h>

/// COMPONENT
#include <QString>

using namespace csapex;

ErrorState::ErrorState()
    : error_(false)
{
}

ErrorState::~ErrorState()
{

}

void ErrorState::setError(bool e, const std::string& msg, ErrorLevel level)
{
    if(!isError() && !e) {
        return;
    }

    // TODO: print if headless
    //std::cerr << msg << std::endl;
    setErrorSilent(e, msg, level);

    errorEvent(error_, msg, level_);
}

void ErrorState::setErrorSilent(bool e, const std::string &msg, ErrorLevel level)
{
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
        error_ = e;
        level_ = level;
        error_msg_ = msg;
    }

    errorChanged(error_);
}

void ErrorState::errorChanged(bool)
{

}

bool ErrorState::isError() const
{
    QMutexLocker lock(&mutex);
    return error_;
}

ErrorState::ErrorLevel ErrorState::errorLevel() const
{
    QMutexLocker lock(&mutex);
    return level_;
}

std::string ErrorState::errorMessage() const
{
    QMutexLocker lock(&mutex);
    return error_msg_;
}

void ErrorState::errorEvent(bool, const std::string &, ErrorLevel)
{

}
