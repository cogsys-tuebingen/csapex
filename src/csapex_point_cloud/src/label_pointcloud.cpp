/// HEADER
#include "label_pointcloud.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <opencv2/opencv.hpp>

CSAPEX_REGISTER_CLASS(csapex::LabelPointCloud, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

LabelPointCloud::LabelPointCloud()
{
}

void LabelPointCloud::process()
{
    PointCloudMessage::Ptr cloud(input_->getMessage<PointCloudMessage>());
    label_msg_ = labels_->getMessage<CvMatMessage>();

    if((label_msg_->value.type() & 7) != CV_16U) {
        throw std::runtime_error("Label matrix must be of type CV_16UC1");
    }

    boost::apply_visitor (PointCloudMessage::Dispatch<LabelPointCloud>(this), cloud->value);
}

void LabelPointCloud::setup()
{
    setSynchronizedInputs(true);
    input_  = addInput<PointCloudMessage>("PointCloud");
    labels_ = addInput<CvMatMessage>("Labels");
    output_ = addOutput<PointCloudMessage>("Labeled PointCloud");
}

namespace implementation {

template<class PointT, class PointS>
struct Copy {
    static inline PointS apply(const PointT& src)
    {
        PointS dst;
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        return dst;
    }
};

template<>
struct Copy<pcl::PointXYZRGBL, pcl::PointXYZRGB> {
    inline static pcl::PointXYZRGBL apply(const pcl::PointXYZRGB& src)
    {
        pcl::PointXYZRGBL dst;
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.r = src.r;
        dst.g = src.g;
        dst.b = src.b;
        return dst;
    }
};

template<class PointT, class PointS>
struct Impl {
    inline static void label(const typename pcl::PointCloud<PointT>::Ptr src,
                             typename pcl::PointCloud<PointS>::Ptr dst,
                             const cv::Mat &labels)
    {
        if(src->height != (uint) labels.rows)
            throw std::runtime_error("PointCloud height != labels matrix height");

        if(src->width != (uint) labels.cols)
            throw std::runtime_error("PointCloud width != labels matrix width");

        dst->height = src->height;
        dst->width  = src->width;
        dst->header = src->header;
        typename pcl::PointCloud<PointT>::const_iterator it = src->begin();
        for(int y = 0 ; y < labels.rows ; ++y) {
            for(int x = 0 ; x < labels.cols ; ++x) {
                dst->push_back(Copy<PointT, PointS>::apply(*it));
                dst->back().label = labels.at<ushort>(y,x);
                ++it;
            }
        }
    }
};

template<class PointT>
struct Label {
    static void apply(const typename pcl::PointCloud<PointT>::Ptr src,
                      PointCloudMessage::Ptr &dst_msg,
                      const cv::Mat &labels)
    {
        pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>);
        Impl<PointT, pcl::PointXYZL>::label(src, cloud, labels);
        dst_msg->value = cloud;
    }
};

template<>
struct Label<pcl::PointXYZRGB> {
    static void apply(const typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr src,
                      PointCloudMessage::Ptr &dst_msg,
                      const cv::Mat &labels)
    {
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
        Impl<pcl::PointXYZRGB, pcl::PointXYZRGBL>::label(src, cloud, labels);
        dst_msg->value = cloud;
    }
};

template<>
struct Label<pcl::PointXY> {
    static void apply(const typename pcl::PointCloud<pcl::PointXY>::Ptr src,
                      PointCloudMessage::Ptr &dst_msg,
                      const cv::Mat &labels)
    {
        throw std::runtime_error("Pointcloud must be of type XYZ!");
    }
};
}

template <class PointT>
void LabelPointCloud::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    PointCloudMessage::Ptr out(new PointCloudMessage);

    implementation::Label<PointT>::apply(cloud, out, label_msg_->value);
    output_->publish(out);
}
