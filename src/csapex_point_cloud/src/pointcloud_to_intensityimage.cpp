/// HEADER
#include "pointcloud_to_intensityimage.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_transform/time_stamp_message.h>
#include <csapex_core_plugins/string_message.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::PointCloudToIntensityImage, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

PointCloudToIntensityImage::PointCloudToIntensityImage()
{
    addTag(Tag::get("PointCloud"));
    addTag(Tag::get("Time"));
}

void PointCloudToIntensityImage::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<PointCloudMessage>("PointCloud");

    output_ = addOutput<CvMatMessage>("Intensity Image");
}

void PointCloudToIntensityImage::process()
{
    PointCloudMessage::Ptr msg(input_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<PointCloudToIntensityImage>(this), msg->value);
}

namespace impl {

template <class PointT>
struct Impl
{
    static void inputCloud(PointCloudToIntensityImage* instance, typename pcl::PointCloud<PointT>::Ptr cloud)
    {
        throw std::runtime_error(std::string("point type '") + type2name(typeid(PointT)) + "' not supported");
    }
};

template <>
struct Impl<pcl::PointXYZI>
{
    static void inputCloud(PointCloudToIntensityImage* instance, typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        instance->inputCloudImpl(cloud);
    }
};
}

template <class PointT>
void PointCloudToIntensityImage::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    impl::Impl<PointT>::inputCloud(this, cloud);
}

void PointCloudToIntensityImage::inputCloudImpl(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    unsigned n = cloud->points.size();

    int cols = cloud->width;
    int rows = n / cols;

    CvMatMessage::Ptr output(new CvMatMessage(enc::mono));
    output->value.create(rows,cols, CV_8U);

    typename pcl::PointCloud<pcl::PointXYZI>::const_iterator pt = cloud->points.begin();
    uchar* data = (uchar*) output->value.data;

    for(unsigned idx = 0; idx < n; ++idx) {
        const pcl::PointXYZI& p = *pt;
        *data = p.intensity;

        ++data;
        ++pt;
    }

    output_->publish(output);
}
