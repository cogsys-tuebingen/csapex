#ifndef QWRAPPER_H
#define QWRAPPER_H

#include <QSpinBox>
#include <QDoubleSpinBox>

namespace QWrapper
{
    class QSpinBoxExt : public QSpinBox {

        Q_OBJECT

    public Q_SLOTS:
        void setRange(int min, int max)
        {
            setMinimum(min);
            setMaximum(max);
        }
    };

    class QDoubleSpinBoxExt : public QDoubleSpinBox {

        Q_OBJECT

    public Q_SLOTS:
        void setRange(double min, double max)
        {
            setMinimum(min);
            setMaximum(max);
        }
    };


}

#endif // QWRAPPER_H
