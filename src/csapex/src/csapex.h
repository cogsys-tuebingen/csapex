#ifndef CSAPEX_H
#define CSAPEX_H

/// SYSTEM
#include <QApplication>
#include <QSplashScreen>

#define DEBUG 0

namespace csapex
{

struct CsApexApp : public QApplication {
    CsApexApp(int& argc, char** argv, bool headless);

    virtual bool notify(QObject* receiver, QEvent* event);
};

struct Main : public QObject {
    Q_OBJECT

public:
    Main(CsApexApp &app);
    ~Main();

    int run();
    int main(bool headless, const std::string &config, const std::string& path_to_bin, const std::vector<std::string>& additional_args);

public Q_SLOTS:
    void showMessage(const QString& msg);

private:
    CsApexApp& app;
    QSplashScreen* splash;
};

}

#endif // CSAPEX_H
