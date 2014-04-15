#ifndef STATISTICAL_OUTLIER_REMOVAL_H
#define STATISTICAL_OUTLIER_REMOVAL_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class StatisticalOutlierRemoval : public Node
{
public:
    StatisticalOutlierRemoval();

    virtual void setup();
    virtual void process();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    ConnectorIn*  input_;
    ConnectorOut* output_;

    void update();

    int     mean_k_;
    bool    keep_organized_;
    bool    negative_;
    double  std_dev_mul_thresh_;
///    float  user_filter_value_; /// ONLY IF NEEDED
};
}
#endif // STATISTICAL_OUTLIER_REMOVAL_H
