#ifndef EXCEPTION_HANDLER_H
#define EXCEPTION_HANDLER_H

/// PROJECT
#include <csapex/core/core_fwd.h>
#include <csapex/utility/exceptions.h>
#include <csapex/csapex_export.h>

class QObject;
class QEvent;

struct CSAPEX_EXPORT AppProxy
{
    virtual bool doNotify(QObject* receiver, QEvent* event) = 0;
};

namespace csapex
{

class CSAPEX_EXPORT ExceptionHandler
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
