#ifndef QDOUBLE_SLIDER_H
#define QDOUBLE_SLIDER_H

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

public Q_SLOTS:
    void scaleValue(int value);
    void setDoubleMinimum(double min);
    void setDoubleMaximum(double max);
    void setDoubleValue(double val);
    double doubleValue();
};


#endif // QDOUBLE_SLIDER_H
