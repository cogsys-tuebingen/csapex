#ifndef QLIMITER_H
#define QLIMITER_H

#include <QObject>
#include <QSlider>
template<class T1, class T2, class DT>
class QLimiter : public QObject
{
    Q_OBJECT
public:
    QLimiter(T1 *limitMin, T2 *limitMax) :
        limitMin_(limitMin),
        limitMax_(limitMax)
    {
        QObject::connect(limitMin_, SIGNAL(valueChanged(DT)), this, SLOT(limitMin(DT)));
        QObject::connect(limitMax_, SIGNAL(valueChanged(DT)), this, SLOT(limitMax(DT)));
    }

    virtual ~QLimiter()
    {
        QObject::disconnect(limitMin_, SIGNAL(valueChanged(DT)), this, SLOT(limitMin(DT)));
        QObject::disconnect(limitMax_, SIGNAL(valueChanged(DT)), this, SLOT(limitMax(DT)));
    }

private Q_SLOTS:
    void limitMin(DT limit)
    {
        DT limited = std::max(limitMax_->value(), limit);
        limitMax_->setValue(limited);
    }

    void limitMax(DT limit)
    {
        DT limited = std::min(limitMin_->value(), limit);
        limitMin_->setValue(limited);
    }

private:
    T1 *limitMin_;
    T2 *limitMax_;
};
typedef QLimiter<QSlider, QSlider, int> QLimiterSlider;

#endif // QLIMITER_H
