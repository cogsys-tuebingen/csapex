#ifndef PASSTHROUGH_H_
#define PASSTHROUGH_H_

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {

class PassThrough : public Node
{
public:
    PassThrough();

    virtual void setup();
    virtual void process();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    void updateBorders();
    void updateFields(const std::vector<std::string> &fields);

private:
    ConnectorIn* input_cloud_;
    ConnectorOut* output_pos_;
    ConnectorOut* output_neg_;

    std::vector<std::string> fields_;
};

}

#endif // PASSTHROUGH_H_
