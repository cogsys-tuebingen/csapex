#ifndef FIT_CONE_H
#define FIT_CONE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_core_plugins/string_message.h>

/// POINT CLOUD
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>

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
    ConnectorOut* out_cloud_;

    int shape_inliers_;

    template <class PointT>
    void findCone(typename pcl::PointCloud<PointT>::Ptr  cloud_in, typename pcl::PointCloud<PointT>::Ptr cloud_extracted, pcl::ModelCoefficients::Ptr coefficients_shape);
    template <class PointT>
    void estimateNormals(typename pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
    template <class PointT>
    void initializeSegmentater(pcl::SACSegmentationFromNormals<PointT, pcl::Normal>  &segmenter);
};

} // namespace csapex

#endif // FIT_CONE_H

