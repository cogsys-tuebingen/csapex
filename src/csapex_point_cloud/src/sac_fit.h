#ifndef SAC_FIT_H
#define SAC_FIT_H

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
class SacFit : public csapex::Node
{
public:
    SacFit();

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

    // PCL parameter
    int ransac_;
    int iterations_;
    double normal_distance_weight_;
    double distance_threshold_;
    double sphere_r_min_;
    double sphere_r_max_;
    pcl::SacModel model_;
    bool publish_inverse_;

    template <class PointT>
    void findModel(typename pcl::PointCloud<PointT>::Ptr  cloud_in, typename pcl::PointCloud<PointT>::Ptr cloud_extracted, pcl::ModelCoefficients::Ptr coefficients_shape);
    template <class PointT>
    void estimateNormals(typename pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
    template <class PointT>
    void initializeSegmenter(pcl::SACSegmentationFromNormals<PointT, pcl::Normal>  &segmenter);
    void setParameters();
};

} // namespace csapex

#endif // SAC_FIT_H

