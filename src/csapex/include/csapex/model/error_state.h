#ifndef DISPLAYABLE_H
#define DISPLAYABLE_H

/// COMPONENT
#include <csapex/csapex_export.h>

/// SYSTEM
#include <string>
#include <mutex>

#undef ERROR

namespace csapex
{

class CSAPEX_EXPORT ErrorState
{
public:
    enum class ErrorLevel {
        NONE = 0,
        ERROR = 1,
        WARNING = 2
    };


public:
    virtual void setError(bool e, const std::string& msg = "", ErrorLevel level = ErrorLevel::ERROR);
    void setErrorSilent(bool e, const std::string& msg = "", ErrorLevel level = ErrorLevel::ERROR);

    bool isError() const;
    ErrorLevel errorLevel() const;
    std::string errorMessage() const;

protected:
    ErrorState();
    virtual ~ErrorState();

    virtual void errorEvent(bool error, const std::string& msg, ErrorLevel level);
    virtual void errorChanged(bool error);

private:
    mutable std::recursive_mutex error_mutex_;
    bool error_;
    std::string error_msg_;
    ErrorLevel level_;
};

}

#endif // DISPLAYABLE_H
