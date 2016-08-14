#ifndef QDOUBLE_SLIDER_H
#define QDOUBLE_SLIDER_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>

/// SYSTEM
#include <QSlider>

class CSAPEX_QT_EXPORT QDoubleSlider : public QSlider
{
    Q_OBJECT

public:
    QDoubleSlider(Qt::Orientation orientation, double step_size, QWidget *parent = 0);
    virtual ~QDoubleSlider();

    void setStepSize(double step);

private:
    double step_;
    double min_;
    double max_;

    double int2double(int val);
    int double2int(double val);

    void update();

Q_SIGNALS:
    void scaledValueChanged(double value);
    void scaledRangeChanged(double min, double max);

public Q_SLOTS:
    void scaleValue(int value);

    void setScaledMinimum(double min);
    void setScaledMaximum(double max);
    void setScaledRange(double min, double max);
    void setScaledValue(double val);
    void setNearestDoubleValue(double val);

    void limitMin(double limit);
    void limitMax(double limit);

    double scaledValue();
    double scaledMaximum();
    double scaledMinimum();

private Q_SLOTS:
    void emitRangeChanged(int min, int max);
};


#endif // QDOUBLE_SLIDER_H
