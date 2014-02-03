#ifndef QDOUBLE_SLIDER_H
#define QDOUBLE_SLIDER_H

/// SYSTEM
#include <QSlider>

class QDoubleSlider : public QSlider
{
    Q_OBJECT

public:
    QDoubleSlider(Qt::Orientation orientation, double step_size, QWidget *parent = 0);
    virtual ~QDoubleSlider();

private:
    double step_;
    double min_;
    double max_;

    double int2double(int val);
    int double2int(double val);

    void update();

Q_SIGNALS:
    void valueChanged(double value);
    void rangeChanged(double min, double max);

public Q_SLOTS:
    void scaleValue(int value);
    void setDoubleMinimum(double min);
    void setDoubleMaximum(double max);
    void setDoubleValue(double val);
    void setNearestDoubleValue(double val);
    void limitMin(double limit);
    void limitMax(double limit);
    double doubleValue();
    double doubleMaximum();
    double doubleMinimum();

private Q_SLOTS:
    void emitRangeChanged(int min, int max);
};


#endif // QDOUBLE_SLIDER_H
