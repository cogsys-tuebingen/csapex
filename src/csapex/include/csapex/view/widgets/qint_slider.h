#ifndef QINT_SLIDER_H
#define QINT_SLIDER_H

/// SYSTEM
#include <QSlider>

class QIntSlider : public QSlider
{
    Q_OBJECT

public:
    QIntSlider(Qt::Orientation orientation, double step_size, QWidget *parent = 0);
    virtual ~QIntSlider();

private:
    int step_;
    int min_;
    int max_;

    int int2integer(int val);
    int integer2int(int val);

    void update();

Q_SIGNALS:
    void scaledValueChanged(int value);
    void scaledRangeChanged(int min, int max);

public Q_SLOTS:
    void scaleValue(int value);
    void setScaledMinimum(int min);
    void setScaledMaximum(int max);
    void setScaledRange(int min, int max);
    void setScaledValue(int val);
    void limitMin(int limit);
    void limitMax(int limit);
    int scaledValue();
    int scaledMaximum();
    int scaledMinimum();

private Q_SLOTS:
    void emitRangeChanged(int min, int max);
};


#endif // QINT_SLIDER_H
