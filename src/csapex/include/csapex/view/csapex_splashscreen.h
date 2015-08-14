#ifndef CSAPEX_SPLASHSCREEN_H
#define CSAPEX_SPLASHSCREEN_H

#include <QSplashScreen>
#include <QApplication>

namespace csapex
{

class CsApexSplashScreen : public QSplashScreen
{
    Q_OBJECT

public:
    explicit CsApexSplashScreen(QWidget *parent = 0);

protected:
    void drawContents(QPainter *painter);

};

}

#endif // CSAPEX_SPLASHSCREEN_H

