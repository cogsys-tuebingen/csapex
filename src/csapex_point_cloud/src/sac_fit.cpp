#include "sac_fit.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_core_plugins/ros_message_conversion.h>
#include <utils_param/parameter_factory.h>
#include <csapex_point_cloud/model_message.h>


/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <boost/mpl/for_each.hpp>
#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::SacFit, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace std;



SacFit::SacFit()
{
    addTag(Tag::get("PointCloud"));

    addParameter(param::ParameterFactory::declare("iterations", 1, 20000, 5000, 200));
    addParameter(param::ParameterFactory::declare("normal distance weight", 0.0, 2.0, 0.085, 0.001));
    addParameter(param::ParameterFactory::declare("distance threshold", 0.0, 2.0, 0.009, 0.001));
    addParameter(param::ParameterFactory::declare("sphere min radius", 0.0, 2.0, 0.02, 0.005));
    addParameter(param::ParameterFactory::declare("sphere max radius", 0.0, 2.0, 0.8, 0.005));
    addParameter(param::ParameterFactory::declare("publish inverse", false));

    std::map<std::string, int> models = boost::assign::map_list_of
            ("fit sphere", (int) pcl::SACMODEL_SPHERE)
            ("fit plane", (int) pcl::SACMODEL_PLANE)
            //("fit cylinder", (int) pcl::SACMODEL_CYLINDER)
            ("fit cone", (int) pcl::SACMODEL_CONE);


    addParameter(param::ParameterFactory::declareParameterSet<int>("models", models),
                 boost::bind(&SacFit::setParameters, this));

}


void SacFit::process()
{
    PointCloudMessage::Ptr msg(input_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<SacFit>(this), msg->value);

    setParameters();
}

void SacFit::setup()
{
    setSynchronizedInputs(true);
    input_ = addInput<PointCloudMessage>("PointCloud");
    out_text_= addOutput<StringMessage>("String");

    out_model_ = addOutput<GenericMessage<ModelMessage> >("Model");
    out_cloud_ = addOutput<PointCloudMessage>("PointCloud");
}


template <class PointT>
void SacFit::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::stringstream stringstream;
    int inliers_size = 0;

    if (out_cloud_->isConnected()) {
        //typename pcl::PointCloud<PointT>::Ptr cloud_extracted;
        //cloud_extracted.reset(new pcl::PointCloud<PointT>);
        typename pcl::PointCloud<PointT>::Ptr cloud_extracted(new pcl::PointCloud<PointT>);
        pcl::ModelCoefficients::Ptr coefficients_shape (new pcl::ModelCoefficients);

        inliers_size = findModel<PointT>(cloud, cloud_extracted, coefficients_shape);



        stringstream << "found [" << shape_inliers_ <<  "] inliers coeffs:" << coefficients_shape->values.at(0) << ", "<< coefficients_shape->values.at(1) << ", "<< coefficients_shape->values.at(2) << ", "<< coefficients_shape->values.at(3) ;
        //stringstream << "Cone apex: "<< coefficients_shape->values[0] << ", " << coefficients_shape->values[1] << ", "<< coefficients_shape->values[2] << ", opening angle: " << coefficients_shape->values[6];

        if (inliers_size > 0) {
            PointCloudMessage::Ptr cloud_msg(new PointCloudMessage);
            cloud_msg->value = cloud_extracted;
            out_cloud_->publish(cloud_msg);

            // Publish the model coefficients of the object
            GenericMessage<ModelMessage>::Ptr param_msg(new GenericMessage<ModelMessage>);
            param_msg->value.reset(new ModelMessage);

            param_msg->value->model_type = model_;
            param_msg->value->coefficients = coefficients_shape;
            param_msg->value->frame_id = cloud->header.frame_id;
            param_msg->value->probability = ransac_probability_;
            out_model_->publish(param_msg);
        }
    } else
    {
        stringstream << "No output cloud connected";
    }

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
int SacFit::findModel(typename pcl::PointCloud<PointT>::Ptr  cloud_in, typename pcl::PointCloud<PointT>::Ptr cloud_extracted, pcl::ModelCoefficients::Ptr coefficients_shape)
{
    //pcl::ExtractIndices<pcl::Normal> extract_normals_;
    pcl::ExtractIndices<PointT> extract_points;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> segmenter;
    initializeSegmenter<PointT>(segmenter);

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    estimateNormals<PointT>(cloud_in, normals);

    // Segment and extract the found points
    segmenter.setInputCloud(cloud_in);
    segmenter.setInputNormals(normals);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    segmenter.segment(*inliers, *coefficients_shape);
    ransac_probability_ = segmenter.getProbability();


    if (inliers->indices.size() > 0) {
        extract_points.setInputCloud(cloud_in);
        extract_points.setIndices(inliers);
        extract_points.setNegative(publish_inverse_);
        extract_points.filter(*cloud_extracted);
    }

    return inliers->indices.size();
}

template <class PointT>
void SacFit::estimateNormals(typename pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    typename pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation;
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    normal_estimation.setSearchMethod (tree);
    normal_estimation.setInputCloud (cloud);
    normal_estimation.setKSearch (50);
    normal_estimation.compute (*normals);
}

void SacFit::setParameters()
{
    //Set Parameters
    iterations_ = param<int>("iterations");
    ransac_ = pcl::SAC_RANSAC;

    normal_distance_weight_ = param<double>("normal distance weight");
    distance_threshold_ = param<double>("distance threshold");
    sphere_r_min_ = param<double>("sphere min radius");
    sphere_r_max_ = param<double>("sphere max radius");

    publish_inverse_ = param<bool>("publish inverse");
    model_ = (pcl::SacModel) param<int>("models");
}

template <class PointT>
void SacFit::initializeSegmenter(pcl::SACSegmentationFromNormals<PointT, pcl::Normal>  &segmenter)
{
    segmenter.setOptimizeCoefficients (true);
    segmenter.setModelType (model_);
    segmenter.setMethodType (ransac_);
    segmenter.setMaxIterations (iterations_);
    segmenter.setDistanceThreshold (distance_threshold_);
    segmenter.setRadiusLimits (sphere_r_min_, sphere_r_max_);
    segmenter.setMinMaxOpeningAngle(sphere_r_min_, sphere_r_max_);
    segmenter.setNormalDistanceWeight (normal_distance_weight_);
    segmenter.setOptimizeCoefficients(true); // optimize the coefficients
}

//
