#ifndef EXCEPTION_HANDLER_H
#define EXCEPTION_HANDLER_H

/// PROJECT
#include <csapex/core/core_fwd.h>
#include <csapex/utility/exceptions.h>

class QObject;
class QEvent;

struct AppProxy
{
    virtual bool doNotify(QObject* receiver, QEvent* event) = 0;
};

namespace csapex
{

class ExceptionHandler
{
public:
    ExceptionHandler(bool fatal_exceptions);
    virtual ~ExceptionHandler();


    void setCore(csapex::CsApexCore* core);

    virtual bool notifyImpl(AppProxy* app, QObject* receiver, QEvent* event);
    virtual void handleAssertionFailure(const csapex::Failure& assertion);

protected:
    void pause();

protected:
    bool fatal_exceptions_;

    csapex::CsApexCore* core_;
};
}

#endif // EXCEPTION_HANDLER_H
