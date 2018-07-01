/// HEADER
#include <csapex/view/gui_exception_handler.h>

/// PROJECT
#include <csapex/model/error_state.h>

/// SYSTEM
#include <QApplication>
#include <QMessageBox>
#include <QDesktopServices>
#include <QUrl>
#include <sstream>

using namespace csapex;

GuiExceptionHandler::GuiExceptionHandler(bool fatal_exceptions) : ExceptionHandler(fatal_exceptions), last_failure_(nullptr)
{
    qRegisterMetaType<Failure>("Failure");

    QObject::connect(this, SIGNAL(fatalError()), this, SLOT(showErrorDialog()));
}

GuiExceptionHandler::~GuiExceptionHandler()
{
    delete last_failure_;
}

void GuiExceptionHandler::handleAssertionFailure(const Failure& failure)
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

    std::string msg = msg_stream.str();
    std::string failure = last_failure_->type();

    int reply = QMessageBox::critical(window, QString::fromStdString(failure), QString::fromStdString(msg), "&Ignore", "&Report (Email)", "&Create Issue");

    switch (reply) {
        case 0:  // ignore
            break;
        case 1:  // report
            reportEmail(failure, msg);
            break;
        case 2:  // issue on github or gitlab
            reportIssue(failure, msg);
            break;
    }
}

void GuiExceptionHandler::reportIssue(const std::string& failure, const std::string& message)
{
    int which = QMessageBox::question(QApplication::activeWindow(), "Please select a target.", "Please choose where to post the new issue:", "github.com", "gitlab.cs.uni-tuebingen.de", 0);

    QString failure_type = QString::fromStdString(failure);
    QString msg = QString::fromStdString(message);

    switch (which) {
        case 0: {
            QString issue_github = "https://github.com/cogsys-tuebingen/csapex/issues/new?title=";
            issue_github += failure_type + "&body=" + msg;
            QDesktopServices::openUrl(QUrl(issue_github, QUrl::TolerantMode));
        } break;

        case 1: {
            QString issue_gitlab = "https://gitlab.cs.uni-tuebingen.de/csapex/csapex/issues/new?issue[assignee_id]=&issue[milestone_id]=&issue[title]=";
            issue_gitlab += failure_type + "&issue[description]=" + msg;
            QDesktopServices::openUrl(QUrl(issue_gitlab, QUrl::TolerantMode));
        } break;
    }
}

void GuiExceptionHandler::reportEmail(const std::string& failure_type, const std::string& msg)
{
    QString mail = "mailto:sebastian.buck@uni-tuebingen.de?";
    mail += "subject=[CS::APEX] Bug Report&body=" + QString::fromStdString(msg);
    QDesktopServices::openUrl(QUrl(mail, QUrl::TolerantMode));
}

/// MOC
#include "../../include/csapex/view/moc_gui_exception_handler.cpp"
