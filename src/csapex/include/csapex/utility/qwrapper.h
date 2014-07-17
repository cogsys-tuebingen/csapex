#ifndef QWRAPPER_H
#define QWRAPPER_H

/// SYSTEM
#include <QSpinBox>
#include <QDoubleSpinBox>

namespace QWrapper
{
class QSpinBoxExt : public QSpinBox {

    Q_OBJECT

public Q_SLOTS:
    void setRange(int min, int max)
    {
        QSpinBox::setRange(min, max);
    }
};

class QDoubleSpinBoxExt : public QDoubleSpinBox {

    Q_OBJECT

public Q_SLOTS:
    void setRange(double min, double max)
    {
        QDoubleSpinBox::setRange(min, max);
    }
};
}

#endif // QWRAPPER_H
