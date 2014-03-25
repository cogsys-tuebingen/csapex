#include "cluster_pointcloud.h"

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

/// PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


CSAPEX_REGISTER_CLASS(csapex::ClusterPointcloud, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace std;


ClusterPointcloud::ClusterPointcloud()
{

    addParameter(param::ParameterFactory::declare("ClusterTolerance", 0.001, 2.0, 0.02, 0.001));
    addParameter(param::ParameterFactory::declare("MinClusterSize", 10, 20000, 100, 200));
    addParameter(param::ParameterFactory::declare("MaxClusterSize", 10, 100000, 25000, 1000));
}

void ClusterPointcloud::process()
{
    PointCloudMessage::Ptr msg(in_cloud_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<ClusterPointcloud>(this), msg->value);

    param_clusterTolerance_ = param<double>("ClusterTolerance");
    param_clusterMinSize_   = param<int>("MinClusterSize");
    param_clusterMaxSize_   = param<int>("MaxClusterSize");
}

void ClusterPointcloud::setup()
{
    setSynchronizedInputs(true);
    in_cloud_ = addInput<PointCloudMessage>("PointCloud");
    out_ = addOutput<GenericVectorMessage, pcl::PointIndices >("Clusters");
}

template <class PointT>
void ClusterPointcloud::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // from http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php

      typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
      tree->setInputCloud (cloud);

      boost::shared_ptr<std::vector<pcl::PointIndices> > cluster_indices(new std::vector<pcl::PointIndices>);
      typename pcl::EuclideanClusterExtraction<PointT> ec;
      ec.setClusterTolerance (0.02); // 2cm
      ec.setMinClusterSize (100);
      ec.setMaxClusterSize (25000);
      ec.setSearchMethod (tree);
      ec.setInputCloud (cloud);
      ec.extract (*cluster_indices);

      //boost::shared_ptr<std::vector<pcl::PointIndices> >  clusters(new std::vector<pcl::PointIndices>);

      out_->publish<GenericVectorMessage, pcl::PointIndices >(cluster_indices);

}
