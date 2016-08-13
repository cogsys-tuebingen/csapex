#ifndef GUI_EXCEPTION_HANDLER_H
#define GUI_EXCEPTION_HANDLER_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>

/// PROJECT
#include <csapex/core/exception_handler.h>

/// SYSTEM
#include <QObject>

namespace csapex
{

class CSAPEX_QT_EXPORT GuiExceptionHandler : public QObject, public ExceptionHandler
{
    Q_OBJECT

public:
    GuiExceptionHandler(bool fatal_exceptions);
    ~GuiExceptionHandler();

Q_SIGNALS:
    void fatalError();

private Q_SLOTS:
    void showErrorDialog();

protected:
    virtual bool notifyImpl(AppProxy* app, QObject* receiver, QEvent* event) override;
    virtual void handleAssertionFailure(const csapex::Failure& assertion) override;

private:
    Failure* last_failure_;
};

}

#endif // GUI_EXCEPTION_HANDLER_H
