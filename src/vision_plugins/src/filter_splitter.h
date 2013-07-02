#ifndef FILTER_SPLITTER_H
#define FILTER_SPLITTER_H

/// COMPONENT
#include <vision_evaluator/filter.h>

namespace vision_evaluator {
class Splitter : public vision_evaluator::BoxedObject
{
    Q_OBJECT

public:
    Splitter();

    virtual void fill(QBoxLayout* layout);

private Q_SLOTS:
    void messageArrived(ConnectorIn *source);

private:
    ConnectorIn *input_;
    std::vector<ConnectorOut*> outputs_;

    int channel_count_;

    void updateOutputs();

};
}
#endif // FILTER_SPLITTER_H
