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
}


template <class PointT>
void FitCone::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    int c = cloud->points.size();
    //number_->display(c);
    //output_->publish(std::string(c));
    std::stringstream stingstream;
    stingstream << c;
    StringMessage::Ptr text_msg(new StringMessage);
    text_msg->value = stingstream.str();
    out_text_->publish(text_msg);


    GenericMessage<NamedMap>::Ptr param_msg(new GenericMessage<NamedMap>);
    out_params_->publish(param_msg);



} //
