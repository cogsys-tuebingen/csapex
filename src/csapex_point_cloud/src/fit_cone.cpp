#include "fit_cone.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_core_plugins/ros_message_conversion.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <boost/mpl/for_each.hpp>

CSAPEX_REGISTER_CLASS(csapex::FitCone, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace std;

struct NamedMap {
    std::map<std::string, double> value;
};

FitCone::FitCone()
{
    addTag(Tag::get("PointCloud"));
}


void FitCone::process()
{
    PointCloudMessage::Ptr msg(input_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<FitCone>(this), msg->value);
}

void FitCone::setup()
{
    setSynchronizedInputs(true);
    input_ = addInput<PointCloudMessage>("PointCloud");
    out_text_= addOutput<StringMessage>("String");

    out_params_ = addOutput<GenericMessage<NamedMap> >("Parameters");
    out_cloud_ = addOutput<PointCloudMessage>("PointCloud");
}


template <class PointT>
void FitCone::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::stringstream stringstream;

    if (out_cloud_->isConnected()) {
        //typename pcl::PointCloud<PointT>::Ptr cloud_extracted;
        //cloud_extracted.reset(new pcl::PointCloud<PointT>);
        typename pcl::PointCloud<PointT>::Ptr cloud_extracted(new pcl::PointCloud<PointT>);
        pcl::ModelCoefficients::Ptr coefficients_shape (new pcl::ModelCoefficients);
        printf("Try to find cone !!! ");
        findCone<PointT>(cloud, cloud_extracted, coefficients_shape);
        //cloud_extracted = cloud; // this works
        PointCloudMessage::Ptr cloud_msg(new PointCloudMessage);
        cloud_msg->value = cloud_extracted;
        out_cloud_->publish(cloud_msg);

        stringstream << "found [" << shape_inliers_ <<  "] inliers";
        //stringstream << "Cone apex: "<< coefficients_shape->values[0] << ", " << coefficients_shape->values[1] << ", "<< coefficients_shape->values[2] << ", opening angle: " << coefficients_shape->values[6];
    } else
    {
        stringstream << "No output cloud connected";
    }

    GenericMessage<NamedMap>::Ptr param_msg(new GenericMessage<NamedMap>);
    out_params_->publish(param_msg);


    StringMessage::Ptr text_msg(new StringMessage);
    text_msg->value = stringstream.str();
    out_text_->publish(text_msg);


// Description of Cone parameters: http://docs.pointclouds.org/1.6.0/classpcl_1_1_sample_consensus_model_cone.html
//    apex.x : the X coordinate of cone's apex
//    apex.y : the Y coordinate of cone's apex
//    apex.z : the Z coordinate of cone's apex
//    axis_direction.x : the X coordinate of the cone's axis direction
//    axis_direction.y : the Y coordinate of the cone's axis direction
//    axis_direction.z : the Z coordinate of the cone's axis direction
//    opening_angle : the cone's opening angle

}

template <class PointT>
void FitCone::findCone(typename pcl::PointCloud<PointT>::Ptr  cloud_in, typename pcl::PointCloud<PointT>::Ptr cloud_extracted, pcl::ModelCoefficients::Ptr coefficients_shape)
{
    //pcl::ExtractIndices<pcl::Normal> extract_normals_;
    pcl::ExtractIndices<PointT> extract_points;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> segmenter;
    initializeSegmentater<PointT>(segmenter);

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    estimateNormals<PointT>(cloud_in, normals);

    // Segment and extract the found points
    segmenter.setInputCloud(cloud_in);
    segmenter.setInputNormals(normals);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    segmenter.segment(*inliers, *coefficients_shape);
    shape_inliers_ = inliers->indices.size();

    extract_points.setInputCloud(cloud_in);
    extract_points.setIndices(inliers);
    extract_points.setNegative(false);
    extract_points.filter(*cloud_extracted);
}

template <class PointT>
void FitCone::estimateNormals(typename pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    typename pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation;
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    normal_estimation.setSearchMethod (tree);
    normal_estimation.setInputCloud (cloud);
    normal_estimation.setKSearch (50);
    normal_estimation.compute (*normals);
}

template <class PointT>
void FitCone::initializeSegmentater(pcl::SACSegmentationFromNormals<PointT, pcl::Normal>  &segmenter)
{
    int ransac = pcl::SAC_RANSAC;
    int iterations = 7000;

    double normal_distance_weight = 0.085;
    double distance_threshold = 0.00998;

    double sphere_r_min_ = 0.01;
    double sphere_r_max_ = 0.8;

    segmenter.setOptimizeCoefficients (true);
    segmenter.setModelType (pcl::SACMODEL_SPHERE);
    segmenter.setMethodType (ransac);
    segmenter.setMaxIterations (iterations);
    segmenter.setDistanceThreshold (distance_threshold);
    segmenter.setRadiusLimits (sphere_r_min_, sphere_r_max_);
    segmenter.setNormalDistanceWeight (normal_distance_weight);
}

//
