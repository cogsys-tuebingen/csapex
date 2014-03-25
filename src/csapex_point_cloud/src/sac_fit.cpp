#include "sac_fit.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_core_plugins/ros_message_conversion.h>
#include <csapex_core_plugins/vector_message.h>
#include <utils_param/parameter_factory.h>
#include <tf/tf.h>

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
    addParameter(param::ParameterFactory::declare("min inliers", 5, 20000, 100, 100));
    addParameter(param::ParameterFactory::declare("normal distance weight", 0.0, 2.0, 0.085, 0.001));
    addParameter(param::ParameterFactory::declare("distance threshold", 0.0, 2.0, 0.009, 0.001));
    addParameter(param::ParameterFactory::declare("sphere min radius", 0.0, 2.0, 0.02, 0.005));
    addParameter(param::ParameterFactory::declare("sphere max radius", 0.0, 2.0, 0.8, 0.005));

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
    out_text_= addOutput<StringMessage>("Debug Info");

    out_model_ = addOutput<GenericVectorMessage, ModelMessage >("Models");
    out_cloud_ = addOutput<PointCloudMessage>("Points of Model");
    out_cloud_residue_ = addOutput<PointCloudMessage>("Residue");
}


template <class PointT>
void SacFit::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::stringstream stringstream;
    int inliers_size = 0;


    typename pcl::PointCloud<PointT>::Ptr cloud_extracted(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_residue(new pcl::PointCloud<PointT>);

    boost::shared_ptr<std::vector<ModelMessage> >  models(new std::vector<ModelMessage>);
    inliers_size = findModel<PointT>(cloud, cloud_extracted, *models, cloud_residue, out_cloud_residue_->isConnected());


    // Publish the found modelcoefficients as a vector
    if (inliers_size > 0) {
        out_model_->publish<GenericVectorMessage, ModelMessage>(models);
        stringstream << "found " << models->size() << " models and " << cloud_extracted->size() <<  "points total";
    } else {
        stringstream << "Zero inliers found";
    }

    // Publish all points that belong to some models
   if (out_cloud_->isConnected()) {
       if (inliers_size > 0 ) {
           PointCloudMessage::Ptr cloud_msg(new PointCloudMessage);
           cloud_msg->value = cloud_extracted;
           out_cloud_->publish(cloud_msg);
       }
    }

    // Publish everything that doesent belong to a model
   if (out_cloud_residue_->isConnected()) {
       PointCloudMessage::Ptr cloud_msg_residue(new PointCloudMessage);
       cloud_msg_residue->value = cloud_residue;
       out_cloud_residue_->publish(cloud_msg_residue);
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
int SacFit::findModel(typename pcl::PointCloud<PointT>::Ptr  cloud_in, typename pcl::PointCloud<PointT>::Ptr cloud_extracted, std::vector<ModelMessage> &models, typename pcl::PointCloud<PointT>::Ptr cloud_resisdue, bool get_resisdue)
{
    // Create functional objects
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> segmenter;
    initializeSegmenter<PointT>(segmenter);
    pcl::ExtractIndices<PointT> extract_points;
    pcl::ExtractIndices<pcl::Normal> extract_normals;

    // Create data objects
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    typename pcl::PointCloud<PointT>::Ptr cloud;
     cloud = cloud_in;
    typename pcl::PointCloud<PointT>::Ptr cloud_inliers (new pcl::PointCloud<PointT>);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_shape (new pcl::ModelCoefficients);

    estimateNormals<PointT>(cloud, normals);

    cloud_extracted->header = cloud_in->header;
    while (cloud->size() > 10) { // TODO: add parameter for min inliers
        std::cout << "!! SIZE CLOUD: " << cloud->size() << std::endl;

        // Segment and extract the found points
        segmenter.setInputCloud(cloud);
        segmenter.setInputNormals(normals);

        segmenter.segment(*inliers, *coefficients_shape);
        ransac_probability_ = segmenter.getProbability();

        std::cout << "!! SIZE INLIER: " << inliers->indices.size() << std::endl;
        if (inliers->indices.size() > min_inliers_) {
            // extract the points that belong to a model
            extract_points.setInputCloud(cloud);
            extract_points.setIndices(inliers);
            extract_points.setNegative(false);
            extract_points.filter(*cloud_inliers);
            // append new found inlier cloud points // TODO: index the points with different colors for different models
            *cloud_extracted += *cloud_inliers;

            // Extract the residue
            extract_points.setInputCloud(cloud);
            extract_points.setIndices(inliers);
            extract_points.setNegative(true);
            extract_points.filter(*cloud);

            // Extract the normals from the residue
            extract_normals.setInputCloud(normals);
            extract_normals.setIndices(inliers);
            extract_normals.setNegative(true);
            extract_normals.filter(*normals);

            // Save Model
            ModelMessage model;
            model.coefficients = coefficients_shape;
            model.probability = ransac_probability_;
            model.frame_id = cloud->header.frame_id;
            model.model_type = model_;
            models.push_back(model);
        } else {
            break;
        }

    }

    cloud_resisdue = cloud;
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
    min_inliers_ = param<int>("min inliers");
    ransac_ = pcl::SAC_RANSAC;

    normal_distance_weight_ = param<double>("normal distance weight");
    distance_threshold_ = param<double>("distance threshold");
    sphere_r_min_ = param<double>("sphere min radius");
    sphere_r_max_ = param<double>("sphere max radius");
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
}

//
