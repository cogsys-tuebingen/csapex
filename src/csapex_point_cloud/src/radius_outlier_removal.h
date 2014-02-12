#ifndef STATISTICAL_OUTLIER_REMOVAL_H
#define STATISTICAL_OUTLIER_REMOVAL_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class RadiusOutlierRemoval : public Node
{
public:
    RadiusOutlierRemoval();

    virtual void setup();
    virtual void allConnectorsArrived();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    ConnectorIn*  input_;
    ConnectorOut* output_;

    void update();

    int     min_neighbours_;
    bool    keep_organized_;
    bool    negative_;
    double  search_radius_;
///    float  user_filter_value_; /// ONLY IF NEEDED
};
}
#endif // STATISTICAL_OUTLIER_REMOVAL_H
