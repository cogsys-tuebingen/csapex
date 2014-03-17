#ifndef FIT_CONE_H
#define FIT_CONE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_core_plugins/string_message.h>
namespace csapex {
class FitCone : public csapex::Node
{
public:
    FitCone();

    virtual void process();
    virtual void setup();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    ConnectorIn* input_;
    ConnectorOut* out_text_;
    ConnectorOut* out_params_;
};

} // namespace csapex

#endif // FIT_CONE_H

