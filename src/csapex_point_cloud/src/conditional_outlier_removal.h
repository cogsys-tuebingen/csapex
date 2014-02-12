#ifndef STATISTICAL_OUTLIER_REMOVAL_H
#define STATISTICAL_OUTLIER_REMOVAL_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class ConditionalOutlierRemoval : public Node
{
public:
    ConditionalOutlierRemoval();

    virtual void setup();
    virtual void allConnectorsArrived();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    enum ConditionType {AND, OR};

    ConnectorIn*  input_;
    ConnectorOut* output_;

    void update();

    ConditionType   type_;
    int             conditions_;
    Eigen::Vector2d x_range_;
    Eigen::Vector2d y_range_;
    Eigen::Vector2d z_range_;
    bool          keep_organized_;


};
}
#endif // STATISTICAL_OUTLIER_REMOVAL_H
