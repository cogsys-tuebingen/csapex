#ifndef GUI_EXCEPTION_HANDLER_H
#define GUI_EXCEPTION_HANDLER_H

/// PROJECT
#include <csapex/core/exception_handler.h>

/// SYSTEM
#include <QObject>

namespace csapex
{

class GuiExceptionHandler : public QObject, public ExceptionHandler
{
    Q_OBJECT

public:
    GuiExceptionHandler(bool fatal_exceptions);

Q_SIGNALS:
    void fatalError(const HardAssertionFailure &assertion);

private Q_SLOTS:
    void showErrorDialog(const HardAssertionFailure &assertion);

protected:
    virtual bool notifyImpl(AppProxy* app, QObject* receiver, QEvent* event) override;
    virtual void handleAssertionFailure(const csapex::HardAssertionFailure& assertion) override;
};

}

#endif // GUI_EXCEPTION_HANDLER_H
