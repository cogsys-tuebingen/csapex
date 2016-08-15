#ifndef CSAPEX_SPLASHSCREEN_H
#define CSAPEX_SPLASHSCREEN_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>

/// SYSTEM
#include <QSplashScreen>
#include <QApplication>

namespace csapex
{

class CSAPEX_QT_EXPORT CsApexSplashScreen : public QSplashScreen
{
    Q_OBJECT

public:
    explicit CsApexSplashScreen(QWidget *parent = 0);

protected:
    void drawContents(QPainter *painter);

};

}

#endif // CSAPEX_SPLASHSCREEN_H

