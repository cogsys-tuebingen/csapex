/// HEADER
#include "passthrough.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <utils_param/parameter_factory.h>
#include <utils_param/interval_parameter.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <boost/mpl/for_each.hpp>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <boost/assign/std.hpp>

CSAPEX_REGISTER_CLASS(csapex::PassThrough, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

PassThrough::PassThrough()
{
    addTag(Tag::get("PointCloud"));

    addParameter(param::ParameterFactory::declareInterval("interval", -100.0, 100.0, 0.0, 100.0, 0.01));

    std::vector<std::string> field;
    field.push_back("x");
    addParameter(param::ParameterFactory::declareParameterStringSet("field", field), boost::bind(&PassThrough::updateBorders, this));
}

void PassThrough::setup()
{
    setSynchronizedInputs(true);

    input_cloud_ = addInput<PointCloudMessage>("PointCloud");

    output_pos_ = addOutput<PointCloudMessage>("cropped PointCloud (+)");
    output_neg_ = addOutput<PointCloudMessage>("cropped PointCloud (-)");
}

void PassThrough::updateBorders()
{
    std::string field = param<std::string>("field");
    param::IntervalParameter::Ptr interv = getParameter<param::IntervalParameter>("interval");

    if(field == "x" || field == "y" || field == "z") {
        interv->setInterval(-10.0, 10.0);
    } else {
        interv->setInterval(0.0, 255.0);
    }
}

void PassThrough::updateFields(const std::vector<std::string>& fields)
{
    if(fields.size() == fields_.size()) {
        return;
    }

    fields_ = fields;

    param::SetParameter::Ptr setp = boost::dynamic_pointer_cast<param::SetParameter>(getParameter("field"));
    if(setp) {
        setError(false);
        std::string old_field = param<std::string>("field");
        setp->setSet(fields);
        setp->set(old_field);
    }
}

void PassThrough::process()
{
    PointCloudMessage::Ptr msg(input_cloud_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<PassThrough>(this), msg->value);
}

template <class PointT>
void PassThrough::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::vector<pcl::PCLPointField> fields;
    std::vector<std::string> field_names;
    pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));

    for(size_t d = 0; d < fields.size (); ++d) {
        field_names.push_back(fields[d].name);
    }

    updateFields(field_names);


    // check available fields!
    pcl::PassThrough<PointT> pass;
    pass.setFilterFieldName (param<std::string>("field"));

    param::IntervalParameter::Ptr interv = getParameter<param::IntervalParameter>("interval");
    pass.setFilterLimits (interv->lower<double>(), interv->upper<double>());
    pass.setInputCloud(cloud);

    if(output_pos_->isConnected()) {
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pass.filter(*out);
        out->header = cloud->header;

        PointCloudMessage::Ptr msg(new PointCloudMessage);
        msg->value = out;
        output_pos_->publish(msg);
    }

    if(output_neg_->isConnected()) {
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pass.setNegative(true);
        pass.filter(*out);
        out->header = cloud->header;

        PointCloudMessage::Ptr msg(new PointCloudMessage);
        msg->value = out;
        output_neg_->publish(msg);
    }

}
