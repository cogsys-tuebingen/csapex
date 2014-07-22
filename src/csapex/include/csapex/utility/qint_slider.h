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
    void intValueChanged(int value);
    void intRangeChanged(int min, int max);

public Q_SLOTS:
    void scaleValue(int value);
    void setIntMinimum(int min);
    void setIntMaximum(int max);
    void setIntRange(int min, int max);
    void setIntValue(int val);
    void limitMin(int limit);
    void limitMax(int limit);
    int intValue();
    int intMaximum();
    int intMinimum();

private Q_SLOTS:
    void emitRangeChanged(int min, int max);
};


#endif // QINT_SLIDER_H
