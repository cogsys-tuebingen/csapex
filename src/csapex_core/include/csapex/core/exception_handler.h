#ifndef EXCEPTION_HANDLER_H
#define EXCEPTION_HANDLER_H

/// PROJECT
#include <csapex/utility/exceptions.h>
#include <csapex_core/csapex_core_export.h>
#include <csapex/utility/slim_signal.hpp>

namespace csapex
{
class CSAPEX_CORE_EXPORT ExceptionHandler
{
public:
    ExceptionHandler(bool fatal_exceptions);
    virtual ~ExceptionHandler();

    virtual bool handleException(std::exception_ptr eptr);
    virtual void handleAssertionFailure(const csapex::Failure& assertion);

public:
    slim_signal::Signal<void()> assertion_failed;

protected:
    void pause();

protected:
    bool fatal_exceptions_;
};
}  // namespace csapex

#endif  // EXCEPTION_HANDLER_H
