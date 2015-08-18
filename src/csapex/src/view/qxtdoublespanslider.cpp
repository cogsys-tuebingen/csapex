/// HEADER
#include <csapex/view/qxtdoublespanslider.h>

/// SYSTEM
#include <assert.h>
#include <iostream>
#include <cmath>

QxtDoubleSpanSlider::QxtDoubleSpanSlider(Qt::Orientation orientation, double step_size, QWidget* parent)
    : QxtSpanSlider(orientation, parent),
      step_(step_size),
      min_(0.0),
      max_(1.0)
{
    //connect(this, SIGNAL(lowerValueChanged(int)), this, SLOT(scaleLowerValue(int)));
    //connect(this, SIGNAL(upperValueChanged(int)), this, SLOT(scaleUpperValue(int)));
    connect(this, SIGNAL(spanChanged(int,int)), this, SLOT(scaleSpan(int,int)));
    connect(this, SIGNAL(rangeChanged(int,int)), this, SLOT(scaleRange(int,int)));
}

void QxtDoubleSpanSlider::setSpan(double lower, double upper)
{
    QxtSpanSlider::setSpan(double2int(lower), double2int(upper));
}

double QxtDoubleSpanSlider::lowerDoubleValue() const
{
    return int2double(lowerValue());
}
double QxtDoubleSpanSlider::upperDoubleValue() const
{
    return int2double(upperValue());
}
double QxtDoubleSpanSlider::doubleMinimum() const
{
    return min_;
}
double QxtDoubleSpanSlider::doubleMaximum() const
{
    return max_;
}

void QxtDoubleSpanSlider::setLowerDoubleValue(double lower)
{
    setLowerValue(double2int(lower));
}
void QxtDoubleSpanSlider::setUpperDoubleValue(double upper)
{
    setUpperValue(double2int(upper));
}

void QxtDoubleSpanSlider::setDoubleMinimum(double min)
{
    update(min, max_);
}
void QxtDoubleSpanSlider::setDoubleMaximum(double max)
{
    update(min_, max);
}

void QxtDoubleSpanSlider::setDoubleRange(double min, double max)
{
    update(min, max);
}


void QxtDoubleSpanSlider::update(double min, double max)
{
    int mini = double2int(min, min);
    int maxi = double2int(max, min);

    if(minimum() != mini || maximum() != maxi) {
        double low = int2double(lowerValue());
        double up = int2double(upperValue());

        setRange(mini, maxi);

        min_ = min;
        max_ = max;

        setLowerValue(double2int(low));
        setUpperValue(double2int(up));
   }
}


void QxtDoubleSpanSlider::scaleLowerValue(int value)
{
//    double val = int2double(value);
//    if(val != lowerDoubleValue()){
//        setLowerDoubleValue(val);
//    }

//    Q_EMIT lowerValueChanged(val);
}

void QxtDoubleSpanSlider::scaleUpperValue(int value)
{
//    double val = int2double(value);
//    if(val != upperDoubleValue()){
//        setUpperDoubleValue(val);
//    }

//    Q_EMIT upperValueChanged(val);
}

void QxtDoubleSpanSlider::scaleSpan(int l, int u)
{
    double low = int2double(l);
    double up = int2double(u);

    if(low != lowerDoubleValue()){
        setLowerDoubleValue(low);
    }
    if(up != upperDoubleValue()){
        setUpperDoubleValue(up);
    }

    Q_EMIT lowerValueChanged(low);
    Q_EMIT upperValueChanged(up);
    Q_EMIT spanChanged(low, up);
}

void QxtDoubleSpanSlider::scaleRange(int l, int u)
{
    double low = int2double(l);
    double up = int2double(u);

    update(low, up);

//    if(low != doubleMinimum()){
//        setDoubleMinimum(low);
//    }
//    if(up != doubleMaximum()){
//        setDoubleMaximum(up);
//    }

    Q_EMIT rangeChanged(low, up);
}

double QxtDoubleSpanSlider::int2double(int val) const
{
    return int2double(val, min_);
}

int QxtDoubleSpanSlider::double2int(double val) const
{
    return double2int(val, min_);
}


double QxtDoubleSpanSlider::int2double(int val, double min) const
{
    return val * step_ + min;
}

int QxtDoubleSpanSlider::double2int(double val, double min) const
{
    return (val - min) / step_;
}

/// MOC
#include "../../include/csapex/view/moc_qxtdoublespanslider.cpp"
