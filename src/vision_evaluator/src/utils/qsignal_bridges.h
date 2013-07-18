#ifndef QSIGNAL_BRIDGES_H
#define QSIGNAL_BRIDGES_H

#include <QObject>
#include <QAbstractSlider>
#include <boost/shared_ptr.hpp>

namespace QSignalBridges
{
class QAbstractSliderLimiter : public QObject {

    Q_OBJECT

public:
    QAbstractSliderLimiter(QAbstractSlider* limitMin, QAbstractSlider* limitMax);
    virtual ~QAbstractSliderLimiter();

    typedef boost::shared_ptr<QAbstractSliderLimiter> Ptr;

private Q_SLOTS:
    void limitMin(int limit);
    void limitMax(int limit);

private:
    QAbstractSlider *limitMin_;
    QAbstractSlider *limitMax_;
};
}
#endif // QSIGNAL_BRIDGES_H



