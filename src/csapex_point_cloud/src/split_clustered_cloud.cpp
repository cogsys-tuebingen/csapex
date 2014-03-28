#include "split_clustered_cloud.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_param/parameter_factory.h>
#include <csapex_core_plugins/ros_message_conversion.h>
#include <csapex_core_plugins/vector_message.h>

/// SYSTEM
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/utility/register_apex_plugin.h>

/// PCL
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>

CSAPEX_REGISTER_CLASS(csapex::SplitClusteredCloud, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

SplitClusteredCloud::SplitClusteredCloud()
{
      addTag(Tag::get("PointCloud"));
}

void SplitClusteredCloud::process()
{
    PointCloudMessage::Ptr msg(input_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<SplitClusteredCloud>(this), msg->value);
}

void SplitClusteredCloud::setup()
{
    setSynchronizedInputs(true);
    input_  = addInput<PointCloudMessage>("PointCloud");
    in_indices_ = addInput <GenericVectorMessage, pcl::PointIndices>("Clusters");

    output1_ = addOutput<PointCloudMessage>("PointCloud1");
    output2_ = addOutput<PointCloudMessage>("PointCloud2");
    output3_ = addOutput<PointCloudMessage>("PointCloud3");
    output4_ = addOutput<PointCloudMessage>("PointCloud4");
}

template <class PointT>
void SplitClusteredCloud::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{

    std::vector<PointCloudMessage::Ptr> out_msgs;
    boost::shared_ptr<std::vector<pcl::PointIndices> const> cluster_indices;
    cluster_indices = in_indices_->getMessage<GenericVectorMessage, pcl::PointIndices>();

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices->begin(); it != cluster_indices->end (); ++it)
    {
        // for every cluster
        // extract the points of the cluster
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cloud_cluster->header = cloud->header;

        PointCloudMessage::Ptr out(new PointCloudMessage);
        out->value = cloud_cluster;
        out_msgs.push_back(out);
    }


    if (out_msgs.size() >= 1) output1_->publish(out_msgs.at(0));
    if (out_msgs.size() >= 2) output2_->publish(out_msgs.at(1));
    if (out_msgs.size() >= 3) output3_->publish(out_msgs.at(2));
    if (out_msgs.size() >= 4) output4_->publish(out_msgs.at(3));
}
