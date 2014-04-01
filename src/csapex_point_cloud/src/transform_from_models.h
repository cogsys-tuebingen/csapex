#ifndef TRANSFORM_FROM_MODELS_H
#define TRANSFORM_FROM_MODELS_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class TransformFromModels : public csapex::Node
{
public:
    TransformFromModels();

    virtual void process();
    virtual void setup();
private:
    ConnectorIn* input_models_ref_;
    ConnectorIn* input_models_new_;
    ConnectorOut* output_;

    // Get interresting points from model

    // Match sides of triangle

    // Find Transformation from matched triangels

    // Optimize transform over time see: http://www.cs.cmu.edu/~ranjith/lcct.html
};
}
#endif // TRANSFORM_FROM_MODELS_H

