/// HEADER
#include "radius_outlier_removal.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_transform/time_stamp_message.h>
#include <csapex_core_plugins/string_message.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

CSAPEX_REGISTER_CLASS(csapex::RadiusOutlierRemoval, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

RadiusOutlierRemoval::RadiusOutlierRemoval() :
    min_neighbours_(2),
    keep_organized_(false),
    negative_(false),
    search_radius_(0.8)
  ///    ,user_filter_value_(std::numeric_limits<float>::quiet_NaN) /// ONLY IF NEEDED
{
    addTag(Tag::get("PointCloud"));

    addParameter(param::ParameterFactory::declareRange("min neighbours", 1, 1000, min_neighbours_, 1),
                 boost::bind(&RadiusOutlierRemoval::update, this));
    addParameter(param::ParameterFactory::declareBool ("keep organized", keep_organized_),
                 boost::bind(&RadiusOutlierRemoval::update, this));
    addParameter(param::ParameterFactory::declareBool ("negate", negative_),
                 boost::bind(&RadiusOutlierRemoval::update, this));
    addParameter(param::ParameterFactory::declareRange("search radius", 0.0, 30.0, search_radius_, 0.1),
                 boost::bind(&RadiusOutlierRemoval::update, this));
}

void RadiusOutlierRemoval::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<PointCloudMessage>("PointCloud");
    output_ = addOutput<PointCloudMessage>("Pointcloud");
}

void RadiusOutlierRemoval::allConnectorsArrived()
{
    PointCloudMessage::Ptr msg(input_->getMessage<PointCloudMessage>());
    boost::apply_visitor (PointCloudMessage::Dispatch<RadiusOutlierRemoval>(this), msg->value);
}

template <class PointT>
void RadiusOutlierRemoval::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(cloud);
    ror.setKeepOrganized(keep_organized_);
    ror.setNegative(negative_);
    ror.setRadiusSearch(search_radius_);
    ror.setMinNeighborsInRadius (min_neighbours_);
    ror.filter(*cloud_filtered);

    PointCloudMessage::Ptr out(new PointCloudMessage);
    out->value = cloud_filtered;
    output_->publish(out);
}

void RadiusOutlierRemoval::update()
{
    min_neighbours_= param<int>("min neighbours");
    keep_organized_= param<bool>("keep organized");
    negative_      = param<bool>("negate");
    search_radius_ = param<double>("search radius");
}
