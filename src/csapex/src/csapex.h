#ifndef CSAPEX_H
#define CSAPEX_H

/// SYSTEM
#include <QApplication>
#include <QSplashScreen>

#define DEBUG 0

namespace csapex
{

struct CsApexApp : public QApplication {
    CsApexApp(int& argc, char** argv);

    virtual bool notify(QObject* receiver, QEvent* event);
};

struct Main : public QObject {
    Q_OBJECT

public:
    Main(CsApexApp& app);

    int run();
    int main(bool headless);

public Q_SLOTS:
    void showMessage(const QString& msg);

private:
    CsApexApp& app;
    QSplashScreen* splash;
};

}

#endif // CSAPEX_H
