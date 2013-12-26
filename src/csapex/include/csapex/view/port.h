#ifndef PORT_H
#define PORT_H

/// SYSTEM
#include <QFrame>

namespace csapex
{

class Port : public QFrame
{
    Q_OBJECT

    Q_PROPERTY(QString class READ cssClass)

public:
    Port();

    QString cssClass() {
        return QString("Port");
    }


protected:
    void paintEvent(QPaintEvent *);
    void refreshStylesheet();

protected:
    bool refresh_style_sheet_;
};

}

#endif // PORT_H
