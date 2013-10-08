#ifndef TRANSFORM_COMBINER_H
#define TRANSFORM_COMBINER_H

/// PROJECT
#include <csapex/model/boxed_object.h>
#include <csapex/utility/qdouble_slider.h>

namespace csapex {

class TransformCombiner : public csapex::BoxedObject
{
    Q_OBJECT

public:
    TransformCombiner();

    virtual void allConnectorsArrived();
    virtual void fill(QBoxLayout* layout);

private:
    ConnectorOut* output_;

    ConnectorIn* input_a_;
    ConnectorIn* input_b_;
};

}

#endif // TRANSFORM_COMBINER_H
