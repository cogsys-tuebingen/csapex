#ifndef QSIGNAL_BRIDGES_H
#define QSIGNAL_BRIDGES_H

/// COMPONENT
#include <csapex_qt/export.h>

/// SYSTEM
#include <QObject>
#include <QAbstractSlider>
#include <memory>

namespace QSignalBridges
{
class CSAPEX_QT_EXPORT QAbstractSliderLimiter : public QObject
{
    Q_OBJECT

public:
    QAbstractSliderLimiter(QAbstractSlider* limitMin, QAbstractSlider* limitMax);
    ~QAbstractSliderLimiter() override;

    typedef std::shared_ptr<QAbstractSliderLimiter> Ptr;

private Q_SLOTS:
    void limitMin(int limit);
    void limitMax(int limit);

private:
    QAbstractSlider* limitMin_;
    QAbstractSlider* limitMax_;
};
}  // namespace QSignalBridges
#endif  // QSIGNAL_BRIDGES_H
