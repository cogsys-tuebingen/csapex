/// HEADER
#include "image_provider_img.h"

REGISTER_IMAGE_PROVIDERS(ImageProviderImg, jpg, png, gif, ppm, pgm)

using namespace csapex;


boost::function<bool(ImageProvider*)> ImageProviderImg::Identity
= boost::bind(&ImageProviderImg::checkIdentity, _1);

ImageProviderImg::ImageProviderImg(const std::string& path)
    : img_(cv::imread(path)), border_(false), displayed(false)
{
}

ImageProvider* ImageProviderImg::createInstance(const std::string& path)
{
    return new ImageProviderImg(path);
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
