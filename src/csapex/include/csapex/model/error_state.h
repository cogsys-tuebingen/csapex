#ifndef DISPLAYABLE_H
#define DISPLAYABLE_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <string>

namespace csapex
{

class ErrorState
{
public:
    typedef int ErrorLevel;

    static const int EL_ERROR = 0;
    static const int EL_WARNING = 1;

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
    virtual void errorChanged(bool error);

protected:
    boost::mutex* error_mutex_;
    bool error_;
    std::string error_msg_;
    ErrorLevel level_;
};

}

#endif // DISPLAYABLE_H
