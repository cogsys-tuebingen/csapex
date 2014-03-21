#ifndef MODEL_MESSAGE_H
#define MODEL_MESSAGE_H

/// PCL
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace csapex
{

class ModelMessage
{
public:
    ModelMessage();
    pcl::SacModel model_type;
    pcl::ModelCoefficients::Ptr coefficients;
    std::string                  frame_id;
    double probability;
};

}

#endif // MODEL_MESSAGE_H
