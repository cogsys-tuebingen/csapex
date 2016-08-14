#ifndef DOUBLESPANSLIDER_H
#define DOUBLESPANSLIDER_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <qxt5/qxtspanslider.h>

class CSAPEX_QT_EXPORT DoubleSpanSlider : public QxtSpanSlider
{
    Q_OBJECT

public:
    explicit DoubleSpanSlider(Qt::Orientation orientation, double step_size, QWidget* parent = 0);

    double lowerDoubleValue() const;
    double upperDoubleValue() const;
    double doubleMinimum() const;
    double doubleMaximum() const;


public Q_SLOTS:
    void setSpan(double lower, double upper);

    void setLowerDoubleValue(double lower);
    void setUpperDoubleValue(double upper);
    void setDoubleMinimum(double min);
    void setDoubleMaximum(double max);

    void setDoubleRange(double min, double max);
    void setRange(double min, double max);

    void scaleSpan(int l, int u);
    void scaleRange(int l, int u);

    void update(double min, double max);

Q_SIGNALS:
    void rangeChanged(double lower, double upper);
    void spanChanged(double lower, double upper);
    void lowerValueChanged(double lower);
    void upperValueChanged(double upper);

private:
    double int2double(int val) const;
    int double2int(double val) const;
    double int2double(int val, double min_) const;
    int double2int(double val, double min_) const;

private:
    double step_;
    double min_;
    double max_;
};

#endif // DOUBLESPANSLIDER_H
