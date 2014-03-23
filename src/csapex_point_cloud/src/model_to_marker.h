#ifndef MODEL_TO_MARKER_H
#define MODEL_TO_MARKER_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/model_message.h>

namespace csapex {
class ModelToMarker : public csapex::Node
{
public:
    ModelToMarker();

    virtual void process();
    virtual void setup();

private:
    ConnectorIn* input_;
    ConnectorOut* output_;

    void publishMarkers(const ModelMessage model_message);
};

} // end namespace csapex
#endif // MODEL_TO_MARKER_H
