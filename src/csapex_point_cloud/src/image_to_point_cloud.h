#ifndef IMAGE_TO_POINT_CLOUD_H
#define IMAGE_TO_POINT_CLOUD_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace cv
{
class Mat;
}

namespace csapex
{

class ImageToPointCloud : public Node
{
public:
    ImageToPointCloud();

    virtual void setup();
    virtual void allConnectorsArrived();

private:
    template <typename PointT>
    connection_types::PointCloudMessage::Ptr
    transform(const cv::Mat& range, const cv::Mat& intensity);

private:
    ConnectorIn* input_depth_;
    ConnectorIn* input_intensity_;
    ConnectorOut* output_;
};

}

#endif // IMAGE_TO_POINT_CLOUD_H
