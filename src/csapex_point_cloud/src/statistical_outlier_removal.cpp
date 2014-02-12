/// HEADER
#include "statistical_outlier_removal.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_transform/time_stamp_message.h>
#include <csapex_core_plugins/string_message.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

CSAPEX_REGISTER_CLASS(csapex::StatisticalOutlierRemoval, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

StatisticalOutlierRemoval::StatisticalOutlierRemoval() :
    mean_k_(2),
    keep_organized_(false),
    negative_(false),
    std_dev_mul_thresh_(0.0)
  ///    ,user_filter_value_(std::numeric_limits<float>::quiet_NaN) /// ONLY IF NEEDED
{
    addTag(Tag::get("PointCloud"));

    addParameter(param::ParameterFactory::declareRange("mean k", 1, 100, mean_k_, 1),
                 boost::bind(&StatisticalOutlierRemoval::update, this));
    addParameter(param::ParameterFactory::declareBool ("keep organized", keep_organized_),
                 boost::bind(&StatisticalOutlierRemoval::update, this));
    addParameter(param::ParameterFactory::declareBool ("negate", negative_),
                 boost::bind(&StatisticalOutlierRemoval::update, this));
    addParameter(param::ParameterFactory::declareRange("std dev threshold", 0.0, 10.0, std_dev_mul_thresh_, 0.1),
                 boost::bind(&StatisticalOutlierRemoval::update, this));
}

void StatisticalOutlierRemoval::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<PointCloudMessage>("PointCloud");
    output_ = addOutput<PointCloudMessage>("Pointcloud");
}

void StatisticalOutlierRemoval::allConnectorsArrived()
{
    PointCloudMessage::Ptr msg(input_->getMessage<PointCloudMessage>());
    boost::apply_visitor (PointCloudMessage::Dispatch<StatisticalOutlierRemoval>(this), msg->value);
}

template <class PointT>
void StatisticalOutlierRemoval::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setKeepOrganized(keep_organized_);
    sor.setMeanK(mean_k_);
    sor.setNegative(negative_);
    sor.setStddevMulThresh(std_dev_mul_thresh_);
    ///    sor.setUserFilterValue(user_filter_value_); /// ONLY IF NEEDED
    sor.filter(*cloud_filtered);

    PointCloudMessage::Ptr out(new PointCloudMessage);
    out->value = cloud_filtered;
    output_->publish(out);
}

void StatisticalOutlierRemoval::update()
{
    mean_k_             = param<int>("mean k");
    keep_organized_     = param<bool>("keep organized");
    negative_           = param<bool>("negate");
    std_dev_mul_thresh_ = param<double>("std dev threshold");
}
