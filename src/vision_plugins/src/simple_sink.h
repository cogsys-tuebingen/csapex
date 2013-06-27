#ifndef SIMPLE_SINK_H
#define SIMPLE_SINK_H

/// COMPONENT
#include <vision_evaluator/image_combiner.h>
#include <designer/connector_in.h>

/// SYSTEM
#include <QLabel>

namespace vision_evaluator
{

class SimpleSink : public vision_evaluator::BoxedObject
{
    Q_OBJECT

public:
    SimpleSink();
    virtual void fill(QBoxLayout* layout);

private Q_SLOTS:
    void messageArrived(ConnectorIn* source);

private:
    ConnectorIn* input_;
    ConnectorOut *output_;

    QLabel* label;
    int sunk;
};

}


#endif // SIMPLE_SINK_H
