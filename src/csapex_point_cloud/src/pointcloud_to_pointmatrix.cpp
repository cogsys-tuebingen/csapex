/// HEADER
#include "pointcloud_to_pointmatrix.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

CSAPEX_REGISTER_CLASS(csapex::PointCloudToPointMatrix, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

PointCloudToPointMatrix::PointCloudToPointMatrix()
{
}

void PointCloudToPointMatrix::process()
{
    PointCloudMessage::Ptr msg(input_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<PointCloudToPointMatrix>(this), msg->value);
}

void PointCloudToPointMatrix::setup()
{
    setSynchronizedInputs(true);
    input_  = addInput<PointCloudMessage>("PointCloud");
    output_ = addOutput<CvMatMessage>("Point Matrix");
    mask_   = addOutput<CvMatMessage>("Vadility");
}

namespace implementation {
template<class PointT>
struct Impl {
    static void convert(const typename pcl::PointCloud<PointT>::Ptr cloud, cv::Mat &matrix, cv::Mat &mask)
    {
        int height = cloud->height;
        int width  = cloud->width;
        matrix = cv::Mat(height, width, CV_32FC3);
        mask   = cv::Mat(height, width, CV_8UC1, 255);

        for(int i = 0 ; i < height ; ++i) {
            for(int j = 0 ; j < width ; ++j) {
                PointT pos = cloud->at(i * width + j);
                matrix.at<float>(i, (j * 3 + 0)) = pos.x;
                matrix.at<float>(i, (j * 3 + 1)) = pos.y;
                matrix.at<float>(i, (j * 3 + 2)) = pos.z;
                if(pos.x == 0.f && pos.y == 0.f && pos.z == 0.f) {
                    mask.at<uchar>(i,j) = 0;
                }
            }
        }
    }
};
template <>
struct Impl<pcl::PointXY> {
    static void convert(const typename pcl::PointCloud<pcl::PointXY>::Ptr cloud, cv::Mat &matrix, cv::Mat &mask)
    {
        std::runtime_error("Conversion is not supported for pcl::PointXY!");
    }
};

template<>
struct Impl<pcl::PointXYZI> {
    static void convert(const typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, cv::Mat &matrix, cv::Mat &mask)
    {
        int height = cloud->height;
        int width  = cloud->width;
        matrix = cv::Mat(height, width, CV_32FC4);
        mask   = cv::Mat(height, width, CV_8UC1, 255);

        for(int i = 0 ; i < height ; ++i) {
            for(int j = 0 ; j < width ; ++j) {
                pcl::PointXYZI pos = cloud->at(i * width + j);
                matrix.at<float>(i, (j * 4 + 0)) = pos.x;
                matrix.at<float>(i, (j * 4 + 1)) = pos.y;
                matrix.at<float>(i, (j * 4 + 2)) = pos.z;
                matrix.at<float>(i, (j * 4 + 3)) = pos.intensity;
                if(pos.x == 0.f && pos.y == 0.f && pos.z == 0.f) {
                    mask.at<uchar>(i,j) = 0;
                }
            }
        }
    }
};

}


template <class PointT>
void PointCloudToPointMatrix::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    #warning "FIX ENCODING"
    CvMatMessage::Ptr out(new CvMatMessage(enc::unknown));
    CvMatMessage::Ptr mask(new CvMatMessage(enc::mono));
    implementation::Impl<PointT>::convert(cloud, out->value, mask->value);
    output_->publish(out);
    mask_->publish(mask);
}
