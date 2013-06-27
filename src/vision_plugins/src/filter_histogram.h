#ifndef FILTER_HISTOGRAM_H
#define FILTER_HISTOGRAM_H

/// COMPONENT
#include <vision_evaluator/filter.h>
namespace vision_evaluator {
class Histogram : public vision_evaluator::BoxedObject
{
    Q_OBJECT

public:
    Histogram();

    virtual void fill(QBoxLayout* layout);

private Q_SLOTS:
    void messageArrived(ConnectorIn* source);

private:
    ConnectorIn  *input_;
    ConnectorOut *output_;
    ConnectorOut *output_histogram_;
};
}
#endif // FILTER_HISTOGRAM_H
