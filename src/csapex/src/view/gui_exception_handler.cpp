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

using namespace csapex;

GuiExceptionHandler::GuiExceptionHandler(bool fatal_exceptions)
    : ExceptionHandler(fatal_exceptions)
{
    qRegisterMetaType < HardAssertionFailure > ("HardAssertionFailure");

    QObject::connect(this, SIGNAL(fatalError(HardAssertionFailure)), this, SLOT(showErrorDialog(HardAssertionFailure)));
}

bool GuiExceptionHandler::notifyImpl(AppProxy* app, QObject *receiver, QEvent *event)
{
    return ExceptionHandler::notifyImpl(app, receiver, event);
}

void GuiExceptionHandler::handleAssertionFailure(const HardAssertionFailure &assertion)
{
    pause();

    assertion.printStackTrace();

    Q_EMIT fatalError(assertion);
}

void GuiExceptionHandler::showErrorDialog(const HardAssertionFailure &assertion)
{
    auto window = QApplication::activeWindow();

    std::stringstream msg;
    if(!assertion.msg.empty()) {
        msg << assertion.msg << "\n";
    }
    msg << "assertion \"" << assertion.code << "\" failed in "
        << assertion.file << ", line " << assertion.line << ", thread \"" << assertion.thread << "\"" << std::endl;

    msg << "\n\n" << assertion.stackTrace(12);

    std::string title = assertion.msg.empty() ? assertion.code : assertion.msg;

    int reply = QMessageBox::critical(window, QString::fromStdString(title),
                                      QString::fromStdString(msg.str()), "&Ignore", "&Report (Email)", "&Create Issue on GitLab");

    switch(reply) {
    case 0: // ignore
        break;
    case 1: // report
    {
        QString mail = "mailto:sebastian.buck@uni-tuebingen.de?";
        mail += "subject=[CS::APEX] Bug Report&body=" + QString::fromStdString(msg.str());
        QDesktopServices::openUrl(QUrl(mail, QUrl::TolerantMode));
    }
        break;
    case 2: // issue
    {
        QString issue = "https://gitlab.cs.uni-tuebingen.de/csapex/csapex/issues/new?issue[assignee_id]=&issue[milestone_id]=&issue[title]=";
        issue += "Assertion Failed&issue[description]=" + QString::fromStdString(msg.str());
        QDesktopServices::openUrl(QUrl(issue, QUrl::TolerantMode));
    }
        break;
    }
}

/// MOC
#include "../../include/csapex/view/moc_gui_exception_handler.cpp"
