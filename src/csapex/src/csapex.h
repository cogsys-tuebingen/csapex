#ifndef CSAPEX_H
#define CSAPEX_H

/// PROJECT
#include <csapex/view/widgets/csapex_splashscreen.h>
#include <csapex/core/csapex_core.h>
#include <csapex/core/settings.h>
#include <csapex/command/command_fwd.h>
#include <csapex/utility/exceptions.h>
#include <csapex/core/exception_handler.h>

/// SYSTEM
#include <QApplication>
#include <memory>

#define DEBUG 0

namespace csapex
{


struct CsApexCoreApp : public QCoreApplication, public AppProxy
{
    CsApexCoreApp(int& argc, char** argv, ExceptionHandler& handler);

    virtual bool notify(QObject* receiver, QEvent* event) override;
    virtual bool doNotify(QObject* receiver, QEvent* event) override;

private:
    ExceptionHandler& handler;
};

struct CsApexGuiApp : public QApplication, public AppProxy
{
    CsApexGuiApp(int& argc, char** argv, ExceptionHandler& handler);

    virtual bool notify(QObject* receiver, QEvent* event) override;
    virtual bool doNotify(QObject* receiver, QEvent* event) override;

    void handleAssertionFailure(const csapex::HardAssertionFailure& assertion);

private:
    ExceptionHandler& handler;
};

struct Main : public QObject {
    Q_OBJECT

public:
    Main(std::unique_ptr<QCoreApplication> &&app, ExceptionHandler &handler);
    ~Main();

    int run();
    int main(bool headless, bool threadless, bool paused, bool thread_grouping, const std::string &config, const std::string& path_to_bin, const std::vector<std::string>& additional_args);

public Q_SLOTS:
    void showMessage(const QString& msg);

private:
    void askForRecoveryConfig(const std::string &config_to_load);
    void deleteRecoveryConfig();

private:
    std::unique_ptr<QCoreApplication> app;
    ExceptionHandler& handler;
    CsApexSplashScreen* splash;

    Settings settings;
};

}

#endif // CSAPEX_H
