#ifndef DISPLAYABLE_H
#define DISPLAYABLE_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <string>
#include <QWidget>
#include <QMutex>

namespace csapex
{

class ErrorState
{
public:
    enum ErrorLevel {
        EL_ERROR, EL_WARNING
    };

public:
    virtual void setError(bool e, const std::string& msg = "", ErrorLevel level = EL_ERROR);
    void setErrorSilent(bool e, const std::string& msg = "", ErrorLevel level = EL_ERROR);

    bool isError() const;
    ErrorLevel errorLevel() const;
    std::string errorMessage() const;

protected:
    ErrorState();
    virtual ~ErrorState();

    virtual void errorEvent(bool error, const std::string& msg, ErrorLevel level);

protected:
    mutable QMutex mutex;
    bool error_;
    std::string error_msg_;
    ErrorLevel level_;
};

}

#endif // DISPLAYABLE_H
