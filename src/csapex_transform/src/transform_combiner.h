#ifndef TRANSFORM_COMBINER_H
#define TRANSFORM_COMBINER_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class TransformCombiner : public csapex::Node
{
    Q_OBJECT

public:
    TransformCombiner();

    virtual void process();
    virtual void setup();

private:
    ConnectorOut* output_;

    ConnectorIn* input_a_;
    ConnectorIn* input_b_;
};

}

#endif // TRANSFORM_COMBINER_H
