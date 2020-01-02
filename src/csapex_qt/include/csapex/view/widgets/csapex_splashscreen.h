#ifndef CSAPEX_SPLASHSCREEN_H
#define CSAPEX_SPLASHSCREEN_H

/// COMPONENT
#include <csapex_qt/export.h>

/// SYSTEM
#include <QSplashScreen>
#include <QApplication>

namespace csapex
{
class CSAPEX_QT_EXPORT CsApexSplashScreen : public QSplashScreen
{
    Q_OBJECT

public:
    explicit CsApexSplashScreen(QWidget* parent = nullptr);

protected:
    void drawContents(QPainter* painter) override;
};

}  // namespace csapex

#endif  // CSAPEX_SPLASHSCREEN_H
