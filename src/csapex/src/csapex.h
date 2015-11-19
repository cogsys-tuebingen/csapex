#ifndef CSAPEX_H
#define CSAPEX_H

/// PROJECT
#include <csapex/view/widgets/csapex_splashscreen.h>
#include <csapex/core/settings.h>
#include <csapex/command/command_fwd.h>

/// SYSTEM
#include <QApplication>
#include <memory>

#define DEBUG 0

namespace csapex
{

struct CsApexCoreApp : public QCoreApplication {
    CsApexCoreApp(int& argc, char** argv, bool fatal_exceptions);

    virtual bool notify(QObject* receiver, QEvent* event);

    bool fatal_exceptions_;
};

struct CsApexApp : public QApplication {
    CsApexApp(int& argc, char** argv, bool fatal_exceptions);

    virtual bool notify(QObject* receiver, QEvent* event);

    bool fatal_exceptions_;
};

struct Main : public QObject {
    Q_OBJECT

public:
    Main(std::unique_ptr<QCoreApplication> &&app);
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
    CsApexSplashScreen* splash;

    Settings settings;
};

}

#endif // CSAPEX_H
