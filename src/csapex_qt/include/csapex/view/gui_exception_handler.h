#ifndef GUI_EXCEPTION_HANDLER_H
#define GUI_EXCEPTION_HANDLER_H

/// COMPONENT
#include <csapex_qt/export.h>

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

    static void reportEmail(const std::string& failure_type, const std::string& msg);
    static void reportIssue(const std::string& failure, const std::string& msg);

Q_SIGNALS:
    void fatalError();

private Q_SLOTS:
    void showErrorDialog();

protected:
    virtual void handleAssertionFailure(const csapex::Failure& assertion) override;

private:
    Failure* last_failure_;
};

}  // namespace csapex

#endif  // GUI_EXCEPTION_HANDLER_H
