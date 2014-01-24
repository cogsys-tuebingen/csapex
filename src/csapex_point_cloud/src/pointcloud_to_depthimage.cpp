/// HEADER
#include "pointcloud_to_depthimage.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_transform/time_stamp_message.h>
#include <csapex_core_plugins/string_message.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::PointCloudToDepthImage, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

PointCloudToDepthImage::PointCloudToDepthImage()
{
    addTag(Tag::get("PointCloud"));
    addTag(Tag::get("Time"));

    addParameter(param::ParameterFactory::declare("scale", 1.0, 1000.0, 1.0, 0.5));
    addParameter(param::ParameterFactory::declare("fit", false));
}

void PointCloudToDepthImage::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<PointCloudMessage>("PointCloud");

    output_ = addOutput<CvMatMessage>("DepthImage");
}

void PointCloudToDepthImage::allConnectorsArrived()
{
    PointCloudMessage::Ptr msg(input_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<PointCloudToDepthImage>(this), msg->value);
}

template <class PointT>
void PointCloudToDepthImage::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    unsigned n = cloud->points.size();

    int cols = cloud->width;
    int rows = n / cols;

    CvMatMessage::Ptr output(new CvMatMessage);
    output->value.create(rows,cols, CV_32F);

    typename pcl::PointCloud<PointT>::const_iterator pt = cloud->points.begin();
    float* data = (float*) output->value.data;

    double max_dist = std::numeric_limits<double>::min();
    double min_dist = std::numeric_limits<double>::max();
    for(unsigned idx = 0; idx < n; ++idx) {
        const PointT& p = *pt;
        double dist = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
        *data = dist;

        if(dist < min_dist) {
            min_dist = dist;
        }
        if(dist > max_dist) {
            max_dist = dist;
        }

        ++data;
        ++pt;
    }

    bool fit = param<bool>("fit");

    double s = param<double>("scale");
    if(fit) {
        s = 255.0 / (max_dist - min_dist);
    }

    output->value = (fit ? (output->value - min_dist) : output->value) * s;

    output_->publish(output);
}
