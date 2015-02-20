#ifndef CSAPEX_H
#define CSAPEX_H

/// SYSTEM
#include <QApplication>
#include <QSplashScreen>
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
    std::unique_ptr<QCoreApplication> app;
    QSplashScreen* splash;
};

}

#endif // CSAPEX_H
