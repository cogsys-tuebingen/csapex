#ifndef TRANSFORM_FROM_MODELS_H
#define TRANSFORM_FROM_MODELS_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/model_message.h>


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

    double param_apex_height_;
    double param_cone_angle_;

    // Get interresting points from model
    std::vector<Eigen::Vector3d> getInterestingPointsFromModels(boost::shared_ptr<std::vector<ModelMessage> const> models);

    // Match sides of triangle
    int matchSidesOfTriangles(const std::vector<Eigen::Vector3d> &points1, const std::vector<Eigen::Vector3d> &points2);
    double euklidianDistance(Eigen::Vector3d p1, Eigen::Vector3d p2);
    // Find Transformation from matched triangels

    // Optimize transform over time see: http://www.cs.cmu.edu/~ranjith/lcct.html
};
}
#endif // TRANSFORM_FROM_MODELS_H

