#ifndef DISPLAYABLE_H
#define DISPLAYABLE_H

/// COMPONENT
#include <csapex/csapex_export.h>
#include <csapex/utility/slim_signal.hpp>

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
    void setError(bool e, const std::string& msg = "", ErrorLevel level = ErrorLevel::ERROR);
    void setErrorSilent(bool e, const std::string& msg = "", ErrorLevel level = ErrorLevel::ERROR);

    virtual bool isError() const;
    virtual ErrorLevel errorLevel() const;
    virtual std::string errorMessage() const;

public:
    slim_signal::Signal<void(bool, const std::string& msg, ErrorLevel level)> error_event;

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
