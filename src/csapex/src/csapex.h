#ifndef CSAPEX_H
#define CSAPEX_H

/// SYSTEM
#include <QApplication>
#include <QSplashScreen>

#define DEBUG 0

namespace csapex
{

struct CsApexApp : public QApplication {
    CsApexApp(int& argc, char** argv, bool headless, bool fatal_exceptions);

    virtual bool notify(QObject* receiver, QEvent* event);

    bool fatal_exceptions_;
};

struct Main : public QObject {
    Q_OBJECT

public:
    Main(CsApexApp &app);
    ~Main();

    int run();
    int main(bool headless, bool threadless, bool thread_grouping, const std::string &config, const std::string& path_to_bin, const std::vector<std::string>& additional_args);

public Q_SLOTS:
    void showMessage(const QString& msg);

private:
    CsApexApp& app;
    QSplashScreen* splash;
};

}

#endif // CSAPEX_H
