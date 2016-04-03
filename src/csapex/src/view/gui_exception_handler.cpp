/// HEADER
#include <csapex/view/gui_exception_handler.h>

/// PROJECT
#include <csapex/core/csapex_core.h>
#include <csapex/model/error_state.h>

/// SYSTEM
#include <QApplication>
#include <QMessageBox>
#include <QDesktopServices>
#include <QUrl>
#include <sstream>

using namespace csapex;

GuiExceptionHandler::GuiExceptionHandler(bool fatal_exceptions)
    : ExceptionHandler(fatal_exceptions), last_failure_(nullptr)
{
    qRegisterMetaType < Failure > ("Failure");

    QObject::connect(this, SIGNAL(fatalError()), this, SLOT(showErrorDialog()));
}

GuiExceptionHandler::~GuiExceptionHandler()
{
    delete last_failure_;
}

bool GuiExceptionHandler::notifyImpl(AppProxy* app, QObject *receiver, QEvent *event)
{
    return ExceptionHandler::notifyImpl(app, receiver, event);
}

void GuiExceptionHandler::handleAssertionFailure(const Failure &failure)
{
    pause();

    failure.printStackTrace();

    delete last_failure_;
    last_failure_ = failure.clone();

    Q_EMIT fatalError();
}

void GuiExceptionHandler::showErrorDialog()
{
    auto window = QApplication::activeWindow();

    std::stringstream msg_stream;
    last_failure_->stackTrace(msg_stream, 12);

    QString msg = QString::fromStdString(msg_stream.str());

    QString failure_type = QString::fromStdString(last_failure_->type());

    int reply = QMessageBox::critical(window, failure_type,
                                      msg, "&Ignore", "&Report (Email)", "&Create Issue on GitLab");

    switch(reply) {
    case 0: // ignore
        break;
    case 1: // report
    {
        QString mail = "mailto:sebastian.buck@uni-tuebingen.de?";
        mail += "subject=[CS::APEX] Bug Report&body=" + msg;
        QDesktopServices::openUrl(QUrl(mail, QUrl::TolerantMode));
    }
        break;
    case 2: // issue
    {
        QString issue = "https://gitlab.cs.uni-tuebingen.de/csapex/csapex/issues/new?issue[assignee_id]=&issue[milestone_id]=&issue[title]=";
        issue += failure_type + "&issue[description]=" + msg;
        QDesktopServices::openUrl(QUrl(issue, QUrl::TolerantMode));
    }
        break;
    }
}

/// MOC
#include "../../include/csapex/view/moc_gui_exception_handler.cpp"
