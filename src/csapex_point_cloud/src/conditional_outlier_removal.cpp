/// HEADER
#include "conditional_outlier_removal.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_transform/time_stamp_message.h>
#include <csapex_core_plugins/string_message.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <boost/assign/list_of.hpp>

CSAPEX_REGISTER_CLASS(csapex::ConditionalOutlierRemoval, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

ConditionalOutlierRemoval::ConditionalOutlierRemoval() :
    type_(AND),
    conditions_(0),
    x_range_(-30.0, 30.0),
    y_range_(-30.0, 30.0),
    z_range_(-30.0, 30.0),
    keep_organized_(false)
{
    addTag(Tag::get("PointCloud"));

    std::map<std::string, int> types = boost::assign::map_list_of
            ("AND", (int) AND)
            ("OR", (int) OR);
    addParameter(param::ParameterFactory::declareParameterSet<int>("type", types), boost::bind(&ConditionalOutlierRemoval::update, this));

    addParameter(param::ParameterFactory::declareBool ("keep organized", keep_organized_),
                 boost::bind(&ConditionalOutlierRemoval::update, this));

    std::map<std::string, int> conditions = boost::assign::map_list_of
            ("x", 1)
            ("y", 2)
            ("z", 4);
    addParameter(param::ParameterFactory::declareParameterBitSet("conditions", conditions),
                 boost::bind(&ConditionalOutlierRemoval::update, this));
    addParameter(param::ParameterFactory::declareRange("min x", -30.0, 30.0, x_range_.x(), 0.1),
                 boost::bind(&ConditionalOutlierRemoval::update, this));
    addParameter(param::ParameterFactory::declareRange("min y", -30.0, 30.0, y_range_.x(), 0.1),
                 boost::bind(&ConditionalOutlierRemoval::update, this));
    addParameter(param::ParameterFactory::declareRange("min z", -30.0, 30.0, z_range_.x(), 0.1),
                 boost::bind(&ConditionalOutlierRemoval::update, this));
    addParameter(param::ParameterFactory::declareRange("max x", -30.0, 30.0, x_range_.y(), 0.1),
                 boost::bind(&ConditionalOutlierRemoval::update, this));
    addParameter(param::ParameterFactory::declareRange("max y", -30.0, 30.0, y_range_.y(), 0.1),
                 boost::bind(&ConditionalOutlierRemoval::update, this));
    addParameter(param::ParameterFactory::declareRange("max z", -30.0, 30.0, z_range_.y(), 0.1),
                 boost::bind(&ConditionalOutlierRemoval::update, this));
}

void ConditionalOutlierRemoval::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<PointCloudMessage>("PointCloud");
    output_ = addOutput<PointCloudMessage>("Filtered Pointcloud");
    update();
}

void ConditionalOutlierRemoval::process()
{
    PointCloudMessage::Ptr msg(input_->getMessage<PointCloudMessage>());
    boost::apply_visitor (PointCloudMessage::Dispatch<ConditionalOutlierRemoval>(this), msg->value);
}

template <class PointT>
void ConditionalOutlierRemoval::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered;
    if(conditions_ != 0) {
        typename pcl::ConditionBase<PointT>::Ptr condition;
        switch(type_) {
        case AND:
            condition.reset(new pcl::ConditionAnd<PointT>);
            break;
        case OR:
            condition.reset(new pcl::ConditionOr<PointT>);
            break;
        default:
            std::runtime_error("Unknown condition type!");
            break;
        }

        if((conditions_ & 1 & 3 & 5) == 1) {
            condition->addComparison (typename pcl::FieldComparison<PointT>::ConstPtr (
                                           new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GT, x_range_.x())));
            condition->addComparison (typename pcl::FieldComparison<PointT>::ConstPtr (
                                           new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT, x_range_.y())));
        }
        if((conditions_ & 2 & 3 & 6) == 2) {
            condition->addComparison (typename pcl::FieldComparison<PointT>::ConstPtr (
                                           new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GT, y_range_.x())));
            condition->addComparison (typename pcl::FieldComparison<PointT>::ConstPtr (
                                           new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT, y_range_.y())));
        }
        if((conditions_ & 4 & 5 & 6) == 4)  {
            condition->addComparison (typename pcl::FieldComparison<PointT>::ConstPtr (
                                           new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, z_range_.x())));
            condition->addComparison (typename pcl::FieldComparison<PointT>::ConstPtr (
                                           new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT, z_range_.y())));
        }
        cloud_filtered.reset(new pcl::PointCloud<PointT>);
        pcl::ConditionalRemoval<PointT> cr (condition);
        cr.setInputCloud(cloud);
        cr.setKeepOrganized(keep_organized_);
        cr.filter(*cloud_filtered);
    } else {
        cloud_filtered.reset(new pcl::PointCloud<PointT>(*cloud));
    }

    PointCloudMessage::Ptr out(new PointCloudMessage);
    out->value = cloud_filtered;
    output_->publish(out);
}

void ConditionalOutlierRemoval::update()
{
    type_ = (ConditionType) param<int>("type");
    conditions_ = param<int>("conditions");
    bool x = (conditions_ & 1 & 3 & 5) == 1;
    bool y = (conditions_ & 2 & 3 & 6) == 2;
    bool z = (conditions_ & 4 & 5 & 6) == 4;
    setParameterEnabled("min x", x);
    setParameterEnabled("max x", x);
    setParameterEnabled("min y", y);
    setParameterEnabled("max y", y);
    setParameterEnabled("min z", z);
    setParameterEnabled("max z", z);

    if(x) {
        x_range_.x() = param<double>("min x");
        x_range_.y() = param<double>("max x");
    }
    if(y) {
        y_range_.x() = param<double>("min y");
        y_range_.y() = param<double>("max y");
    }
    if(z)  {
        z_range_.x() = param<double>("min z");
        z_range_.y() = param<double>("max z");
    }


}
