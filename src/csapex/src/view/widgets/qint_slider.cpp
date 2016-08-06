/// HEADER
#include <csapex/view/widgets/qint_slider.h>

/// SYSTEM
#include <iostream>
#include <limits>

QIntSlider::QIntSlider(Qt::Orientation orientation, double step_size, QWidget *parent) :
    QSlider(orientation, parent),
    step_(step_size),
    min_(0),
    max_(1)
{
    connect(this, SIGNAL(valueChanged(int)), this, SLOT(scaleValue(int)));
    connect(this, SIGNAL(rangeChanged(int,int)), this, SLOT(emitRangeChanged(int,int)));
}

QIntSlider::~QIntSlider()
{
}

void QIntSlider::setStepSize(int step)
{
    step_ = step;
    update();
}

void QIntSlider::update()
{
    if(min_ > max_) {
        max_ = min_;
    }
    int min = integer2int(min_);
    int max = integer2int(max_);

    setSingleStep(step_);
    if(max != maximum() || min != minimum()) {
        setRange(min, max);
    }
}

int QIntSlider::int2integer(int val)
{
    return val * step_ + min_;
}

int QIntSlider::integer2int(int val)
{
    return (val - min_) / step_;
}


void QIntSlider::scaleValue(int value)
{
    double val = int2integer(value);
    if(val != scaledValue()){
        setScaledValue(val);
    }

    Q_EMIT scaledValueChanged(val);
}

void QIntSlider::setScaledMinimum(int min)
{
    bool change = min != min_;

    if(min_ > max_) {
        max_ = min_;
    }

    if(change) {
        min_ = min;
        update();
    }
}

void QIntSlider::setScaledMaximum(int max)
{
    bool change = max != max_;

    if(min_ > max_) {
        min_ = max_;
    }

    if(change) {
        max_ = max;
        update();
    }
}

void QIntSlider::setScaledRange(int min, int max)
{
    bool change = max != max_ || min != min_;

    if(change) {
        min_ = min;
        max_ = max;
        update();
    }
}

void QIntSlider::setScaledValue(int val)
{
    int intval= integer2int(val);
    if(value() != intval) {
        setValue(intval);
    }
}

void QIntSlider::limitMin(int limit)
{
    double limited = std::max(scaledValue(), limit);
    setScaledValue(limited);
}

void QIntSlider::limitMax(int limit)
{
    double limited = std::min(scaledValue(), limit);
    setScaledValue(limited);
}


int QIntSlider::scaledValue()
{
    return int2integer(value());
}

int QIntSlider::scaledMaximum()
{
    return int2integer(maximum());
}

int QIntSlider::scaledMinimum()
{
    return int2integer(minimum());
}

void QIntSlider::emitRangeChanged(int min, int max)
{
    if(int2integer(min) != min_)
        setScaledMinimum(int2integer(min));
    if(int2integer(max) != max_)
        setScaledMaximum(int2integer(max));

    Q_EMIT scaledRangeChanged(min_, max_);
}
/// MOC
#include "../../../include/csapex/view/widgets/moc_qint_slider.cpp"
