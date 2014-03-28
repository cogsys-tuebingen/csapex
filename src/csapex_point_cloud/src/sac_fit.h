#ifndef SAC_FIT_H
#define SAC_FIT_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_core_plugins/string_message.h>
#include <csapex_point_cloud/model_message.h>

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
    ConnectorIn* in_indices_;
    ConnectorOut* out_text_;
    ConnectorOut* out_model_;
    ConnectorOut* out_cloud_;
    ConnectorOut* out_cloud_residue_;

    int shape_inliers_;
    double ransac_probability_;

    // PCL parameter
    int ransac_;
    int iterations_;
    int min_inliers_;
    double normal_distance_weight_;
    double distance_threshold_;
    double sphere_r_min_;
    double sphere_r_max_;
    pcl::SacModel model_;

    boost::shared_ptr<std::vector<pcl::PointIndices> const> cluster_indices_;

    template <class PointT>
    int findModels(typename pcl::PointCloud<PointT>::Ptr  cloud_in, typename pcl::PointCloud<PointT>::Ptr cloud_extracted, std::vector<ModelMessage> &models, typename pcl::PointCloud<PointT>::Ptr cloud_resisdue, bool get_resisdue);
    template <class PointT> // Was "findModel in commit 51178f0ca   fixed bug that it publishes the resiues
    int findSingleModel(typename pcl::PointCloud<PointT>::Ptr  cloud_in, typename pcl::PointCloud<PointT>::Ptr cloud_extracted, pcl::ModelCoefficients::Ptr coefficients_shape, typename pcl::PointCloud<PointT>::Ptr cloud_resisdue, bool get_resisdue);
    template <class PointT>
    void estimateNormals(typename pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
    template <class PointT>
    void initializeSegmenter(pcl::SACSegmentationFromNormals<PointT, pcl::Normal>  &segmenter);
    void setParameters();
};

} // namespace csapex

#endif // SAC_FIT_H

