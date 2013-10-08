#ifndef TRANSFORM_INVERTER_H
#define TRANSFORM_INVERTER_H

/// PROJECT
#include <csapex/model/boxed_object.h>
#include <csapex/utility/qdouble_slider.h>

namespace csapex {

class TransformInverter : public csapex::BoxedObject
{
    Q_OBJECT

public:
    TransformInverter();

    virtual void allConnectorsArrived();
    virtual void fill(QBoxLayout* layout);

private:
    ConnectorIn* input_;
    ConnectorOut* output_;
};

}

#endif // TRANSFORM_INVERTER_H
