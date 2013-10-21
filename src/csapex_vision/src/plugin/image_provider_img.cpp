/// HEADER
#include "image_provider_img.h"

/// SYSTEM
#include <boost/assign.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::ImageProviderImg, csapex::MessageProvider)


using namespace csapex;


boost::function<bool(ImageProvider*)> ImageProviderImg::Identity
= boost::bind(&ImageProviderImg::checkIdentity, _1);

ImageProviderImg::ImageProviderImg()
    : border_(false), displayed(false)
{
}

void ImageProviderImg::load(const std::string& path)
{
    img_ = cv::imread(path, CV_LOAD_IMAGE_UNCHANGED);
}

std::vector<std::string> ImageProviderImg::getExtensions() const
{
    return boost::assign::list_of<std::string> (".jpg")(".jpeg")(".gif")(".png")(".tiff")(".pgm")(".ppm");
}

bool ImageProviderImg::checkIdentity(ImageProvider* other)
{
    return dynamic_cast<ImageProviderImg*>(other) != NULL;
}

void ImageProviderImg::next(cv::Mat& img, cv::Mat& mask)
{
    displayed = true;

    if(border_) {
        int bx = 40;
        int by = 40;

        img = cv::Mat(img_.rows + 2 * bx, img_.cols + 2 * by, img_.type(), cv::Scalar::all(0));

        cv::Rect roi(bx, by, img_.cols, img_.rows);

        img_.copyTo(cv::Mat(img, roi));
        mask = cv::Mat(img.rows, img.cols, CV_8UC1, cv::Scalar::all(0));

        cv::rectangle(mask, roi + cv::Point(2,2) + cv::Size(-4,-4), cv::Scalar::all(255), CV_FILLED);

    } else {
        img_.copyTo(img);
    }
}

void ImageProviderImg::enableBorder(bool border)
{
    border_ = border;
}

bool ImageProviderImg::hasNext()
{
    return true;//!displayed;
}
