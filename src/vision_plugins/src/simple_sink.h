#ifndef SIMPLE_SINK_H
#define SIMPLE_SINK_H

/// COMPONENT
#include <vision_evaluator/image_combiner.h>
#include <designer/connector_in.h>

/// SYSTEM
#include <QLabel>

namespace vision_plugins
{

class SimpleSink : public vision_evaluator::BoxedObject
{
    Q_OBJECT

public:
    SimpleSink();
    virtual void fill(QBoxLayout* layout);

private Q_SLOTS:
    void messageArrived(vision_evaluator::ConnectorIn* source);

private:
    vision_evaluator::ConnectorIn* input_;
    vision_evaluator::ConnectorOut *output_;

    QLabel* label;
    int sunk;
};

}


#endif // SIMPLE_SINK_H
